set(LIBHELLO_SRC hello.c)

add_library(hello SHARED ${LIBHELLO_SRC})
add_library(hello_static STATIC ${LIBHELLO_SRC})

set_target_properties(hello_static PROPERTIES OUTPUT_NAME "hello")

get_target_property(OUTPUT_VALUE hello_static OUTPUT_NAME)
message(STATUS "This is the hello_static OUTPUT_NAME: " ${OUTPUT_VALUE}) 

set_target_properties(hello PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(hello_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

# 设置 动态库 的版本号(VERSION 指代动态库版本，SOVERSION 指代 API 版本)：
set_target_properties(hello PROPERTIES VERSION 1.2 SOVERSION 1)

install(TARGETS hello hello_static LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(FILES hello.h DESTINATION include/hello)

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
