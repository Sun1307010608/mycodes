
# include_directories(/usr/include/hello)

# terminal run command: export CMAKE_INCLUDE_PATH=/usr/include/hello
find_path(myHeader hello.h)  # e.g: FIND_PATH(myHeader NAMES hello.h PATHS /usr/include/usr/include/hello)
if(myHeader)
include_directories(${myHeader})
endif(myheader)

add_executable(main main.c)

# target_link_libraries(main hello)
# target_link_libraries(main libhello.so)
target_link_libraries(main libhello.a)
