###############################################################################
## NAME            : src/CMakeLists.txt
## PURPOSE         : Defines the executable target with all components.
## AUTO-GENERATED  : Adaptive AUTOSAR Code Generation Pipeline
###############################################################################

set(SOURCES
    main.cpp
    ProbeComm/ProbeComm.cpp
    ProbeCommVariant/ProbeCommVariant.cpp
    CanConsumer/CanConsumer.cpp
    ProbeApp/ProbeApp.cpp
)

add_executable(ADprobe__uid__1
    ${SOURCES}
)

target_include_directories(ADprobe__uid__1
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ProbeComm
    ${CMAKE_CURRENT_SOURCE_DIR}/ProbeCommVariant
    ${CMAKE_CURRENT_SOURCE_DIR}/CanConsumer
    ${CMAKE_CURRENT_SOURCE_DIR}/ProbeApp
)

target_link_libraries(ADprobe__uid__1
    PRIVATE
    ${ARA_LIBRARIES}
    ${ARAExec_LIBRARIES}
    ${ara-com_LIBRARIES}
    ${ara-log_LIBRARIES}
    ${ara-per_LIBRARIES}
    Threads::Threads
)

target_compile_options(ADprobe__uid__1
    PRIVATE
    -std=c++14
)

install(
    TARGETS ADprobe__uid__1
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
