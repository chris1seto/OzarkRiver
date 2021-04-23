add_library(Cmsis STATIC "${CMAKE_CURRENT_SOURCE_DIR}/dummy.c")
target_include_directories(Cmsis PUBLIC "${CMAKE_CURRENT_LIST_DIR}/")