cmake_minimum_required(VERSION 3.10)

project(uvpackit)

# Create a variable to allow developers to set the path to the downloaded sdk.
# no default seeing as it doesn't install to any pre defined location.
set(LXSDK_PATH "" CACHE PATH "Path to root of downloaded LXSDK")

# Paths to version 2.5.7 of installed UVPackmaster
set(UVP_INCLUDE "C:/Program Files/UVPackmaster/SDK/std/2.5.8/include" CACHE PATH "Path to UVP includes")
set(UVP_LIBRARY "C:/Program Files/UVPackmaster/SDK/std/2.5.8/lib/vs2019/Release" CACHE PATH "Path to UVP libraries")

# Get all source and headers for lxsdk
# TODO: Read somewhere it's recommended not to use GLOB like this,
file(GLOB LXSDK_SOURCES ${LXSDK_PATH}/common/*.cpp)
file(GLOB LXSDK_HEADERS ${LXSDK_PATH}/include/*.h?)

# CRT_SECURE_NO_WARNINGS on windows,
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# Should create our library so we can now focus on our own project.
add_library(lxsdk STATIC ${LXSDK_SOURCES})
set_target_properties(lxsdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY lib)
target_include_directories(lxsdk PRIVATE ${LXSDK_PATH}/include)

# This is the plug-in we create, shared makes it on windows to a .dll which is
# what we expect for a plug-in
add_library(uvpackit SHARED "source/uvpackit.cpp")

# We also must include the headers for the sdk and uv packmaster
target_include_directories(uvpackit PRIVATE ${LXSDK_PATH}/include)
target_include_directories(uvpackit PRIVATE ${UVP_INCLUDE})

target_link_libraries(uvpackit lxsdk)
target_link_libraries(uvpackit "${UVP_LIBRARY}/uvpcore.lib")

# as the lib for uv packmaster is only available for 64bit windows or unix
# we will first only support windows 64bit systems.
# 
# we could otherwise here potentially later branch for unix also
set(PLUGIN_DIR "win64")

# Set the output to the folder Modo will search, 
# $<0:> is just to remove any config subfolders like DEBUG and RELEASE
set_target_properties(uvpackit 
  PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/${PLUGIN_DIR}/$<0:>
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/${PLUGIN_DIR}/$<0:> # windows apparently needs this set as well
)

# copy the UV Packmaster .dll to the plug-in folder also,
add_custom_command(TARGET uvpackit POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${UVP_LIBRARY}/uvpcore.dll ${PROJECT_SOURCE_DIR}/${PLUGIN_DIR}/$<0:>
)
