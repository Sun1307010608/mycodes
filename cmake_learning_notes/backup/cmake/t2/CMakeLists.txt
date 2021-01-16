project(hello)  # PROJECT(projectname [CXX] [C] [Java])

# add_subdirectory(src bin)  # ADD_SUBDIRECTORY(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
add_subdirectory(src)

# set(SRC_LIST main.c)  # SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])

# # MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"...)
# # SEND_ERROR, 产生错误，生成过程被跳过。
# # STATUS, 输出前缀为—的信息。
# # FATAL_ERROR, 立即终止所有 cmake 过程.
# message(STATUS "This is BINARY dir " ${hello_BINARY_DIR})  # 隐式定义变量（<projectname>区分大小写）： <projectname>_BINARY_DIR
# message(STATUS "This is SOURCE dir " ${hello_SOURCE_DIR})  # 隐式定义变量（<projectname>区分大小写）： <projectname>_SOURCE_DIR
message(STATUS "This is BINARY dir " ${PROJECT_BINARY_DIR})  # 隐式定义变量（推荐）： PROJECT_BINARY_DIR
message(STATUS "This is SOURCE dir " ${PROJECT_SOURCE_DIR})  # 隐式定义变量（推荐）： PROJECT_SOURCE_DIR
message(STATUS "This is INSTALL PREFIX dir: " ${CMAKE_INSTALL_PREFIX})
# add_executable(hello ${SRC_LIST})


INSTALL(FILES COPYRIGHT README DESTINATION share/doc/cmake/t2)
INSTALL(PROGRAMS runhello.sh bin/hello DESTINATION bin)
INSTALL(DIRECTORY doc/ DESTINATION share/doc/cmake/t2)