target_sources_ifdef(CONFIG_ZMK_TWIST_ROUTER app PRIVATE src/twist_router.c)
# Flash is ~99% full on this board; size-optimise this file.
set_source_files_properties(src/twist_router.c PROPERTIES COMPILE_FLAGS "-Os")
