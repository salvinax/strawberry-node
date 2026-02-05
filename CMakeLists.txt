cmake_minimum_required(VERSION 3.20.0)
set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/modules/HX711")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})



project(sensor_node)



target_sources(app PRIVATE
  src/main.c
  src/drv_ads1015.c
  src/drv_mlx90614.c
  src/drv_analog.c
  src/ble_lys.c
  src/drv_sen0546.c
  src/drv_load_point.c
)
