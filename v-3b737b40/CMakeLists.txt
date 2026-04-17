###############################################################################
## HEADER
## NAME            : CMakeLists
## BASE CLASSES    :
## PURPOSE         : Top-level CMake build configuration.
##                   Sets flags for SOME/IP / IPC communication bindings,
##                   compiler options, and AUTOSAR package discovery.
## AUTO-GENERATED  : Adaptive AUTOSAR Code Generation Pipeline
###############################################################################

cmake_minimum_required(VERSION 3.4)

project(ADprobe VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_THREAD_LIBS_INIT "-pthread")

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
        "Choose the type of build, one of: Debug, Release, RelWithDebInfo, MinSizeRel"
        FORCE)
endif()

###############################################################################
# Compiler flags (MISRA / AUTOSAR compliant warnings)
###############################################################################
add_compile_options(
    -pedantic
    -Wextra
    -Wsign-promo
    -Wnon-virtual-dtor
    -Wctor-dtor-privacy
    -Woverloaded-virtual
    -Wold-style-cast
    -Wpointer-arith
    -Wunused
    -Wuninitialized
    -Winit-self
    -Wdeprecated
    -Wfloat-equal
    -fno-strict-aliasing
)

include(GNUInstallDirs)

###############################################################################
# Build options
###############################################################################
option(ENABLE_LIB_RELEASE_BUILD "Enable Data file log for Release build" OFF)

if(ENABLE_LIB_RELEASE_BUILD)
    option(HANDLE_ZAT_TRIGGER_EVT_DATA_FILE "Enable saving ZAT HDF TRIGGER data to file" OFF)
    option(HANDLE_CAMIMG_EVT_DATA_FILE      "Enable saving camera trigger event data to file" OFF)
    option(COLLECT_CAMIMG_DATA_FILE         "Enable collecting camera image data to file" OFF)
    option(COLLECT_DAQ_EVT_DATA_FILE        "Enable collecting DAQ event data to file" OFF)
endif()

option(RUNNING_ON_RCAR "Disabling APIs that cannot be used on RCAR" OFF)
option(LOG_OUTPUT       "Enable Logs"         OFF)
option(TRACELOG         "Enable Tracelogs"    OFF)
option(TRIGGER_LOG      "Enable trigger logs" OFF)

if(LOG_OUTPUT)
    option(LOG_PERFORMANCE_MEASUREMENT "Enable Logs to measure execution time" OFF)
    option(ERRLOG       "Enable error logs"   OFF)
    option(LOGGER       "Enable logger logs"  OFF)
    option(ENABLE_DATA_LOG "Enable data logs" OFF)
endif()

###############################################################################
# Compile definitions from options
###############################################################################
if(ENABLE_LIB_RELEASE_BUILD)
    add_compile_definitions(ENABLE_LIB_RELEASE_BUILD)
    if(HANDLE_ZAT_TRIGGER_EVT_DATA_FILE)
        add_compile_definitions(HANDLE_ZAT_TRIGGER_EVT_DATA_FILE)
    endif()
    if(HANDLE_CAMIMG_EVT_DATA_FILE)
        add_compile_definitions(HANDLE_CAMIMG_EVT_DATA_FILE)
    endif()
    if(COLLECT_CAMIMG_DATA_FILE)
        add_compile_definitions(COLLECT_CAMIMG_DATA_FILE)
    endif()
    if(COLLECT_DAQ_EVT_DATA_FILE)
        add_compile_definitions(COLLECT_DAQ_EVT_DATA_FILE)
    endif()
endif()

if(RUNNING_ON_RCAR)
    add_compile_definitions(RUNNING_ON_RCAR)
endif()

if(LOG_OUTPUT)
    add_compile_definitions(LOG_OUTPUT)
    if(LOG_PERFORMANCE_MEASUREMENT)
        add_compile_definitions(LOG_PERFORMANCE_MEASUREMENT)
    endif()
    if(ERRLOG)
        add_compile_definitions(ERRLOG)
    endif()
    if(LOGGER)
        add_compile_definitions(LOGGER)
    endif()
    if(ENABLE_DATA_LOG)
        add_compile_definitions(ENABLE_DATA_LOG)
    endif()
endif()

if(TRACELOG)
    add_compile_definitions(TRACELOG)
endif()

if(TRIGGER_LOG)
    add_compile_definitions(TRIGGER_LOG)
endif()

###############################################################################
# Communication bindings (SOME/IP + IPC)
###############################################################################

# SOME/IP binding
add_definitions(-DHAS_SOMEIP_BINDING -DIF_IAM_ENABLED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_SOMEIP_BINDING")
set(ARA_LIBRARIES ${ARA_LIBRARIES} ara_someipbinding someip dlt)

# IPC binding
add_definitions(-DHAS_IPC_BINDING -DIF_IAM_ENABLED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_IPC_BINDING")
set(ARA_LIBRARIES ${ARA_LIBRARIES} ara_ipcbinding dlt)

###############################################################################
# Platform / package discovery
###############################################################################
if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    find_package(Threads REQUIRED)
endif()

find_package(ARAExec           REQUIRED)
find_package(ara-com            REQUIRED)
find_package(ara-log            REQUIRED)
find_package(ara-arxmls         REQUIRED)
find_package(ara-per            REQUIRED)
find_package(someip             REQUIRED)
find_package(cdk-cli            REQUIRED)
find_package(AdaptivePlatform   REQUIRED)

###############################################################################
# Source subdirectory
###############################################################################
add_subdirectory(src)
