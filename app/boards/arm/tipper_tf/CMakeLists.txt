set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_BASE}/../tools/uf2/utils/uf2conv.py
  -c 
  -b 0x1000
  -f 0xADA52840
  -o ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.uf2
  ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.bin
)

target_sources_ifdef(CONFIG_CUSTOM_WIDGET_BATTERY_STATUS app PRIVATE battery_status.c)

zephyr_library()
zephyr_library_include_directories(${ZEPHYR_LVGL_MODULE_DIR})
zephyr_library_include_directories(${ZEPHYR_BASE}/lib/gui/lvgl/)
zephyr_library_sources(bat_100.c)
zephyr_library_sources(bat_100_c.c)
zephyr_library_sources(bat_80.c)
zephyr_library_sources(bat_80_c.c)
zephyr_library_sources(bat_60.c)
zephyr_library_sources(bat_60_c.c)
zephyr_library_sources(bat_40.c)
zephyr_library_sources(bat_40_c.c)
zephyr_library_sources(bat_20.c)
zephyr_library_sources(bat_20_c.c)
zephyr_library_sources(bat_00.c)
zephyr_library_sources(bat_00_c.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_DISPLAY tipper_tf_status_screen.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_DISPLAY tipper_tf_logo.c)
zephyr_library_sources(${ZEPHYR_BASE}/misc/empty_file.c)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
