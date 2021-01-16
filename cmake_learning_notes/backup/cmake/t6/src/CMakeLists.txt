
find_package(HELLO)  # HELLO 要大写；

IF(HELLO_FOUND)
    ADD_EXECUTABLE(hello main.c)
    # message(STATUS "hello include dir: " ${HELLO_INCLUDE_DIR})
    # message(STATUS "hello library dir: " ${HELLO_LIBRARY})
    INCLUDE_DIRECTORIES(${HELLO_INCLUDE_DIR})
    TARGET_LINK_LIBRARIES(hello ${HELLO_LIBRARY})
ENDIF(HELLO_FOUND)