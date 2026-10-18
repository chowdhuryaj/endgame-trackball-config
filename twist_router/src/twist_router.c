/*
 * Twist / cursor router (AJ, 2026-10-17).
 *
 * Input processor with a small runtime rule table. Each rule has conditions
 * (modifiers, host lock LEDs, active layer) and one action; first match wins,
 * no match = event passes through untouched.
 *
 *   mode 0 (twist):  acts on INPUT_REL_WHEEL only (the mixer's twist output).
 *   mode 1 (cursor): acts on INPUT_REL_X / INPUT_REL_Y only.
 *
 * Placement in the keymap decides what the event is (twist vs drag-scroll):
 * the twist node sits before any xy->scroll mapping, the cursor node is kept
 * out of the drag-scroll layer chain.
 *
 * Rules persist as settings "twr/<idx>" (twist) and "crr/<idx>" (cursor).
 * Shell: twist rule ..., cursor rule ...
 */

#define DT_DRV_COMPAT zmk_input_processor_twist_router

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <drivers/input_processor.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

#include <zmk/hid.h>
#include <zmk/hid_indicators.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define TWR_MAX_RULES 8

enum { MODE_TWIST = 0, MODE_CURSOR = 1, MODE_COUNT };
enum { MODS_ANY = 0, MODS_ALL, MODS_NONE };
enum { LOCKS_ON = 0, LOCKS_OFF };

/* Action ids are per mode; index into the name tables below. */
enum { TA_SCROLL = 0, TA_REVERSE, TA_HSCROLL, TA_KEYS, TA_BLOCK, TA_COUNT };
enum { CA_SPEED = 0, CA_AXIS, CA_INVERT, CA_SCROLL, CA_BLOCK, CA_COUNT };

static const char *const action_names[MODE_COUNT][5] = {
    {"scroll", "reverse", "hscroll", "keys", "block"},
    {"speed", "axis", "invert", "scroll", "block"},
};
/* Number of args each action takes. */
static const uint8_t action_argc[MODE_COUNT][5] = {
    {1, 1, 2, 3, 0},
    {1, 1, 2, 1, 0},
};
static const char *const mode_prefix[MODE_COUNT] = {"twr", "crr"};
static const char *const mods_mode_names[] = {"any", "all", "none"};
static const char *const locks_mode_names[] = {"on", "off"};

/* Stored as-is in settings; keep the layout stable. */
struct twr_rule {
    uint8_t used;
    uint8_t mods;
    uint8_t mods_mode;
    uint8_t locks;
    uint8_t locks_mode;
    int8_t layer;
    uint8_t action;
    uint8_t reserved;
    uint32_t a[3];
};

/* ponytail: rules are written from the shell thread and read from the input
 * thread without a lock; a rule being edited mid-event can apply half-written
 * once. Add a spinlock if that ever matters. */
static struct twr_rule rules[MODE_COUNT][TWR_MAX_RULES];

/* Per-mode runtime state (one node per mode is expected). */
struct twr_state {
    int8_t last_rule;
    int32_t rem[2];  /* scaling remainders, x100 units: [0]=X/HWHEEL, [1]=Y/WHEEL */
    int32_t key_acc; /* twist keys: accumulated wheel units */
};
static struct twr_state states[MODE_COUNT];


struct twr_config {
    uint8_t mode;
};

/* ---------- conditions ---------- */

static uint8_t locks_now(void) { return (uint8_t)zmk_hid_indicators_get_current_profile(); }

static bool rule_matches(const struct twr_rule *r, uint8_t mods, uint8_t locks) {
    switch (r->mods_mode) {
    case MODS_ANY:
        if (r->mods && !(mods & r->mods)) {
            return false;
        }
        break;
    case MODS_ALL:
        if ((mods & r->mods) != r->mods) {
            return false;
        }
        break;
    case MODS_NONE:
        if (mods & r->mods) {
            return false;
        }
        break;
    }

    if (r->locks_mode == LOCKS_ON ? (locks & r->locks) != r->locks : (locks & r->locks) != 0) {
        return false;
    }

    return r->layer < 0 || zmk_keymap_layer_active((zmk_keymap_layer_id_t)r->layer);
}

static int find_match(uint8_t mode) {
    bool any = false;
    for (int i = 0; i < TWR_MAX_RULES; i++) {
        any |= rules[mode][i].used;
    }
    if (!any) {
        return -1;
    }

    const uint8_t mods = zmk_hid_get_explicit_mods();
    const uint8_t locks = locks_now();
    for (int i = 0; i < TWR_MAX_RULES; i++) {
        if (rules[mode][i].used && rule_matches(&rules[mode][i], mods, locks)) {
            return i;
        }
    }
    return -1;
}

/* ---------- actions ---------- */

static int32_t scale(int32_t v, uint32_t pct, int32_t *rem) {
    const int32_t total = v * (int32_t)pct + *rem;
    const int32_t out = total / 100;
    *rem = total - out * 100;
    return out;
}

/* Drop an event without losing a frame sync another axis may be waiting on. */
static int drop(struct input_event *event) {
    if (!event->sync) {
        return ZMK_INPUT_PROC_STOP;
    }
    event->value = 0;
    return ZMK_INPUT_PROC_CONTINUE;
}

/* Twist "keys": taps are queued from a work item, not the input thread
 * (input thread stack is 1 KiB and a key press runs the whole HID path). */
static struct k_spinlock tap_lock;
static uint32_t tap_usage[2];
static uint8_t tap_count[2];

static void tap_work_cb(struct k_work *work) {
    for (int dir = 0; dir < 2; dir++) {
        k_spinlock_key_t key = k_spin_lock(&tap_lock);
        const uint32_t usage = tap_usage[dir];
        uint8_t n = tap_count[dir];
        tap_count[dir] = 0;
        k_spin_unlock(&tap_lock, key);

        const struct zmk_behavior_binding binding = {
            .behavior_dev = DEVICE_DT_NAME(DT_NODELABEL(kp)),
            .param1 = usage,
        };
        const struct zmk_behavior_binding_event ev = {
            .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(0, 0),
            .timestamp = k_uptime_get(),
        };
        while (n--) {
            if (zmk_behavior_queue_add(&ev, binding, true, CONFIG_ZMK_TWIST_ROUTER_TAP_MS) < 0 ||
                zmk_behavior_queue_add(&ev, binding, false, CONFIG_ZMK_TWIST_ROUTER_TAP_MS) < 0) {
                break; /* queue full: drop the rest */
            }
        }
    }
}
static K_WORK_DEFINE(tap_work, tap_work_cb);

static void queue_tap(int dir, uint32_t usage) {
    if (!usage) {
        return;
    }
    k_spinlock_key_t key = k_spin_lock(&tap_lock);
    if (tap_usage[dir] != usage) {
        tap_usage[dir] = usage;
        tap_count[dir] = 0;
    }
    if (tap_count[dir] < CONFIG_ZMK_TWIST_ROUTER_MAX_PENDING_TAPS) {
        tap_count[dir]++;
    }
    k_spin_unlock(&tap_lock, key);
    k_work_submit(&tap_work);
}

static int twist_apply(const struct twr_rule *r, struct twr_state *st, struct input_event *ev) {
    switch (r->action) {
    case TA_SCROLL:
        ev->value = scale(ev->value, r->a[0], &st->rem[1]);
        return ZMK_INPUT_PROC_CONTINUE;
    case TA_REVERSE:
        ev->value = -scale(ev->value, r->a[0], &st->rem[1]);
        return ZMK_INPUT_PROC_CONTINUE;
    case TA_HSCROLL: {
        const int32_t v = scale(ev->value, r->a[0], &st->rem[0]);
        ev->code = INPUT_REL_HWHEEL;
        ev->value = r->a[1] ? -v : v;
        return ZMK_INPUT_PROC_CONTINUE;
    }
    case TA_KEYS: {
        const int32_t step = (int32_t)r->a[2];
        if ((st->key_acc > 0 && ev->value < 0) || (st->key_acc < 0 && ev->value > 0)) {
            st->key_acc = 0; /* direction change starts fresh */
        }
        st->key_acc += ev->value;
        while (st->key_acc >= step) {
            st->key_acc -= step;
            queue_tap(0, r->a[0]);
        }
        while (st->key_acc <= -step) {
            st->key_acc += step;
            queue_tap(1, r->a[1]);
        }
        return ZMK_INPUT_PROC_STOP; /* twist WHEEL is its own synced frame */
    }
    case TA_BLOCK:
    default:
        return drop(ev);
    }
}

static int cursor_apply(const struct twr_rule *r, struct twr_state *st, struct input_event *ev) {
    const bool is_x = ev->code == INPUT_REL_X;
    int32_t *rem = &st->rem[is_x ? 0 : 1];

    switch (r->action) {
    case CA_SPEED:
        ev->value = scale(ev->value, r->a[0], rem);
        return ZMK_INPUT_PROC_CONTINUE;
    case CA_AXIS: /* 0 = horizontal only, 1 = vertical only */
        if ((r->a[0] == 0) != is_x) {
            return drop(ev);
        }
        return ZMK_INPUT_PROC_CONTINUE;
    case CA_INVERT:
        if (is_x ? r->a[0] : r->a[1]) {
            ev->value = -ev->value;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    case CA_SCROLL:
        /* Same direction convention as the drag-scroll layer: Y is inverted. */
        if (is_x) {
            ev->code = INPUT_REL_HWHEEL;
            ev->value = scale(ev->value, r->a[0], rem);
        } else {
            ev->code = INPUT_REL_WHEEL;
            ev->value = -scale(ev->value, r->a[0], rem);
        }
        return ZMK_INPUT_PROC_CONTINUE;
    case CA_BLOCK:
    default:
        return drop(ev);
    }
}

static int twr_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                            uint32_t param2, struct zmk_input_processor_state *state) {
    const struct twr_config *cfg = dev->config;
    const uint8_t mode = cfg->mode;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (mode == MODE_TWIST ? event->code != INPUT_REL_WHEEL
                           : (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const int idx = find_match(mode);
    struct twr_state *st = &states[mode];
    if (idx != st->last_rule) {
        st->last_rule = idx;
        st->rem[0] = st->rem[1] = 0;
        st->key_acc = 0;
    }
    if (idx < 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return mode == MODE_TWIST ? twist_apply(&rules[mode][idx], st, event)
                              : cursor_apply(&rules[mode][idx], st, event);
}

static const struct zmk_input_processor_driver_api twr_api = {
    .handle_event = twr_handle_event,
};

static int twr_init(const struct device *dev) {
    const struct twr_config *cfg = dev->config;
    states[cfg->mode].last_rule = -1;
    return 0;
}

#define TWR_INST(n)                                                                                \
    BUILD_ASSERT(DT_INST_PROP(n, mode) < MODE_COUNT, "twist-router: mode must be 0 or 1");          \
    static const struct twr_config twr_config_##n = {.mode = DT_INST_PROP(n, mode)};               \
    DEVICE_DT_INST_DEFINE(n, twr_init, NULL, NULL, &twr_config_##n, POST_KERNEL,                    \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &twr_api);

DT_INST_FOREACH_STATUS_OKAY(TWR_INST)

/* ---------- settings ---------- */

static int load_rule(uint8_t mode, const char *name, size_t len, settings_read_cb read_cb,
                     void *cb_arg) {
    char *end;
    const long idx = strtol(name, &end, 10);
    if (end == name || *end != '\0' || idx < 0 || idx >= TWR_MAX_RULES ||
        len != sizeof(struct twr_rule)) {
        return -EINVAL;
    }
    struct twr_rule r;
    const int rc = read_cb(cb_arg, &r, sizeof(r));
    if (rc < 0) {
        return rc;
    }
    rules[mode][idx] = r;
    return 0;
}

static int twr_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *arg) {
    return load_rule(MODE_TWIST, name, len, read_cb, arg);
}
static int crr_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *arg) {
    return load_rule(MODE_CURSOR, name, len, read_cb, arg);
}
SETTINGS_STATIC_HANDLER_DEFINE(twist_router_twr, "twr", NULL, twr_settings_set, NULL, NULL);
SETTINGS_STATIC_HANDLER_DEFINE(twist_router_crr, "crr", NULL, crr_settings_set, NULL, NULL);

static int save_rule(uint8_t mode, int idx) {
    char key[12];
    snprintf(key, sizeof(key), "%s/%d", mode_prefix[mode], idx);
    if (!rules[mode][idx].used) {
        return settings_delete(key);
    }
    return settings_save_one(key, &rules[mode][idx], sizeof(struct twr_rule));
}

/* ---------- shell ---------- */

#if IS_ENABLED(CONFIG_SHELL)

static void print_rule(const struct shell *sh, uint8_t mode, int idx) {
    const struct twr_rule *r = &rules[mode][idx];
    const bool hex = mode == MODE_TWIST && r->action == TA_KEYS;
    char a0[12], a1[12];
    snprintf(a0, sizeof(a0), hex ? "0x%x" : "%u", r->a[0]);
    snprintf(a1, sizeof(a1), hex ? "0x%x" : "%u", r->a[1]);
    shell_print(sh, "R %d 0x%02x %s %u %s %d %s %s %s %u", idx, r->mods,
                mods_mode_names[r->mods_mode], r->locks, locks_mode_names[r->locks_mode], r->layer,
                action_names[mode][r->action], a0, a1, r->a[2]);
}

static bool parse_ul(const char *s, int base, unsigned long *out) {
    char *end;
    if (s[0] == '-') {
        return false;
    }
    *out = strtoul(s, &end, base);
    return end != s && *end == '\0';
}

static int parse_idx(const struct shell *sh, const char *s) {
    unsigned long v;
    if (!parse_ul(s, 10, &v) || v >= TWR_MAX_RULES) {
        shell_print(sh, "error: idx must be 0-%d", TWR_MAX_RULES - 1);
        return -1;
    }
    return (int)v;
}

static int lookup(const char *const *names, int n, const char *s) {
    for (int i = 0; i < n; i++) {
        if (strcmp(names[i], s) == 0) {
            return i;
        }
    }
    return -1;
}

static bool usage_ok(uint32_t u) {
    const uint8_t page = ZMK_HID_USAGE_PAGE(u);
    return u == 0 || page == HID_USAGE_KEY || page == HID_USAGE_CONSUMER;
}

#define ERR(...)                                                                                   \
    do {                                                                                           \
        shell_print(sh, "error: " __VA_ARGS__);                                                    \
        return -EINVAL;                                                                            \
    } while (0)

static int rule_list(const struct shell *sh, uint8_t mode) {
    int n = 0;
    for (int i = 0; i < TWR_MAX_RULES; i++) {
        if (rules[mode][i].used) {
            print_rule(sh, mode, i);
            n++;
        }
    }
    shell_print(sh, "rules: %d/%d", n, TWR_MAX_RULES);
    return 0;
}

/* argv[0]="set", then: idx mods mods_mode locks locks_mode layer action [a1] [a2] [a3] */
static int rule_set(const struct shell *sh, uint8_t mode, size_t argc, char **argv) {
    if (argc < 8) {
        ERR("too few args");
    }

    const int idx = parse_idx(sh, argv[1]);
    if (idx < 0) {
        return -EINVAL;
    }

    struct twr_rule r = {.used = 1};
    unsigned long v;

    if (!parse_ul(argv[2], 16, &v) || v > 0xFF) {
        ERR("mods must be hex 00-ff");
    }
    r.mods = v;

    const int mm = lookup(mods_mode_names, ARRAY_SIZE(mods_mode_names), argv[3]);
    if (mm < 0) {
        ERR("mods mode must be any|all|none");
    }
    r.mods_mode = mm;

    if (!parse_ul(argv[4], 0, &v) || v > 7) {
        ERR("locks must be 0-7");
    }
    r.locks = v;

    const int lm = lookup(locks_mode_names, ARRAY_SIZE(locks_mode_names), argv[5]);
    if (lm < 0) {
        ERR("locks mode must be on|off");
    }
    r.locks_mode = lm;

    char *end;
    const long layer = strtol(argv[6], &end, 10);
    if (end == argv[6] || *end != '\0' || layer < -1 || layer >= ZMK_KEYMAP_LAYERS_LEN) {
        ERR("layer must be -1..%d", ZMK_KEYMAP_LAYERS_LEN - 1);
    }
    r.layer = (int8_t)layer;

    const int act = lookup(action_names[mode], 5, argv[7]);
    if (act < 0) {
        ERR("action must be %s", mode == MODE_TWIST ? "scroll|reverse|hscroll|keys|block"
                                                    : "speed|axis|invert|scroll|block");
    }
    r.action = act;

    const size_t nargs = argc - 8;
    if (nargs != action_argc[mode][act]) {
        ERR("%s takes %d arg(s)", argv[7], action_argc[mode][act]);
    }
    for (size_t i = 0; i < nargs; i++) {
        if (!parse_ul(argv[8 + i], 0, &v) || v > UINT32_MAX) {
            ERR("bad number '%s'", argv[8 + i]);
        }
        r.a[i] = (uint32_t)v;
    }

    /* Per-action range checks. */
    const bool pct_first = (mode == MODE_TWIST && act != TA_KEYS && act != TA_BLOCK) ||
                           (mode == MODE_CURSOR && (act == CA_SPEED || act == CA_SCROLL));
    if (pct_first && (r.a[0] < 1 || r.a[0] > 1000)) {
        ERR("pct must be 1-1000");
    }
    if (mode == MODE_TWIST && act == TA_HSCROLL && r.a[1] > 1) {
        ERR("rev must be 0|1");
    }
    if (mode == MODE_TWIST && act == TA_KEYS) {
        if (!usage_ok(r.a[0]) || !usage_ok(r.a[1])) {
            ERR("bad usage");
        }
        if (r.a[2] < 1 || r.a[2] > 1000) {
            ERR("step must be 1-1000");
        }
    }
    if (mode == MODE_CURSOR && act == CA_AXIS && r.a[0] > 1) {
        ERR("axis must be 0|1");
    }
    if (mode == MODE_CURSOR && act == CA_INVERT && (r.a[0] > 1 || r.a[1] > 1)) {
        ERR("invert must be 0|1");
    }

    rules[mode][idx] = r;
    const int rc = save_rule(mode, idx);
    if (rc < 0) {
        ERR("save failed %d", rc);
    }
    print_rule(sh, mode, idx);
    return 0;
}

static int rule_del(const struct shell *sh, uint8_t mode, size_t argc, char **argv) {
    if (argc != 2) {
        ERR("usage: del <idx>");
    }
    const int idx = parse_idx(sh, argv[1]);
    if (idx < 0) {
        return -EINVAL;
    }
    rules[mode][idx].used = 0;
    save_rule(mode, idx);
    shell_print(sh, "ok");
    return 0;
}

static int rule_clear(const struct shell *sh, uint8_t mode) {
    for (int i = 0; i < TWR_MAX_RULES; i++) {
        if (rules[mode][i].used) {
            rules[mode][i].used = 0;
            save_rule(mode, i);
        }
    }
    shell_print(sh, "ok");
    return 0;
}

static int rule_test(const struct shell *sh, uint8_t mode) {
    const int idx = find_match(mode);
    const zmk_keymap_layer_id_t top = zmk_keymap_layer_index_to_id(zmk_keymap_highest_layer_active());
    char m[12] = "";
    if (idx >= 0) {
        snprintf(m, sizeof(m), "%d", idx);
    }
    shell_print(sh, "mods=0x%02x locks=0x%02x layer=%d match=%s", zmk_hid_get_explicit_mods(),
                locks_now(), top,
                idx >= 0 ? m : "none");
    return 0;
}

#define MODE_CMDS(name, mode)                                                                      \
    static int cmd_##name##_list(const struct shell *sh, size_t argc, char **argv) {               \
        return rule_list(sh, mode);                                                                \
    }                                                                                              \
    static int cmd_##name##_set(const struct shell *sh, size_t argc, char **argv) {                \
        return rule_set(sh, mode, argc, argv);                                                     \
    }                                                                                              \
    static int cmd_##name##_del(const struct shell *sh, size_t argc, char **argv) {                \
        return rule_del(sh, mode, argc, argv);                                                     \
    }                                                                                              \
    static int cmd_##name##_clear(const struct shell *sh, size_t argc, char **argv) {              \
        return rule_clear(sh, mode);                                                               \
    }                                                                                              \
    static int cmd_##name##_test(const struct shell *sh, size_t argc, char **argv) {               \
        return rule_test(sh, mode);                                                                \
    }                                                                                              \
    SHELL_STATIC_SUBCMD_SET_CREATE(sub_##name##_rule,                                              \
                                   SHELL_CMD(list, NULL, "List rules", cmd_##name##_list),         \
                                   SHELL_CMD(set, NULL, "Set a rule", cmd_##name##_set),           \
                                   SHELL_CMD(del, NULL, "Delete a rule", cmd_##name##_del),        \
                                   SHELL_CMD(clear, NULL, "Delete all rules", cmd_##name##_clear), \
                                   SHELL_CMD(test, NULL, "Show state and match", cmd_##name##_test), \
                                   SHELL_SUBCMD_SET_END);                                          \
    SHELL_STATIC_SUBCMD_SET_CREATE(sub_##name, SHELL_CMD(rule, &sub_##name##_rule, "Rules", NULL), \
                                   SHELL_SUBCMD_SET_END);                                          \
    SHELL_CMD_REGISTER(name, &sub_##name, #name " router", NULL);

MODE_CMDS(twist, MODE_TWIST)
MODE_CMDS(cursor, MODE_CURSOR)

#endif /* CONFIG_SHELL */
