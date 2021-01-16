
# 方法1：
# include_directories(/usr/include)

# add_executable(curltest main.c)

# target_link_libraries(curltest curl)



# 方法2：
add_executable(curltest main.c)

find_package(CURL)
if(CURL_FOUND)
    include_directories(${CURL_INCLUDE_DIR})
    target_link_libraries(curltest ${CURL_LIBRARY})
else(CURL_FOUND)
    message(FATAL_ERROR "CURL library not found")
endif(CURL_FOUND)
