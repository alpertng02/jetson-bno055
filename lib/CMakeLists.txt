add_library(libi2c INTERFACE)

target_sources(libi2c INTERFACE ${CMAKE_CURRENT_LIST_DIR}/libi2c/src/i2c.c)

target_include_directories(libi2c INTERFACE ${CMAKE_CURRENT_LIST_DIR}/libi2c/include)


add_library(jetson-bno055 INTERFACE)

target_sources(jetson-bno055 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/bno055.cpp ${CMAKE_CURRENT_LIST_DIR}/BNO055_SensorAPI/bno055.c ${CMAKE_CURRENT_LIST_DIR}/BNO055_SensorAPI/bno055.h) 

target_include_directories(jetson-bno055 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(jetson-bno055 INTERFACE libi2c)