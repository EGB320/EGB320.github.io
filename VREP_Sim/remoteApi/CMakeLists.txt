cmake_minimum_required(VERSION 3.5.1) 

# SET PROJECT
set(PROJECT_TARGET RemoteApi)
project(${PROJECT_TARGET})

add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DDO_NOT_USE_SHARED_MEMORY -DARM_PROCESSOR)

# SET COMPILE FLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c -g")


# GET FILES
file(GLOB SOURCE_FILES ${CMAKE_SOURCE_DIR}/*.c)
file(GLOB HEADER_FILES_1 ${CMAKE_SOURCE_DIR}/*.h)
#file(GLOB HEADER_FILES_2 ${CMAKE_SOURCE_DIR}/../include/*.h)
#message(${HEADER_FILES_2})


# BUILD LIBRARY
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build/lib)
add_library(${PROJECT_TARGET} SHARED ${SOURCE_FILES} ${HEADER_FILES_2})#${HEADER_FILES_1})