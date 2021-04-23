file(GLOB_RECURSE SOURCES "./*.c")
add_library(FreeRTOS STATIC ${SOURCES})
target_include_directories(FreeRTOS PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(FreeRTOS Platform)