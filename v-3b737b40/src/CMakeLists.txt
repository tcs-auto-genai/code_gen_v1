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

add_executable(ADprobe
    ${SOURCES}
)

target_include_directories(ADprobe
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ProbeComm
    ${CMAKE_CURRENT_SOURCE_DIR}/ProbeCommVariant
    ${CMAKE_CURRENT_SOURCE_DIR}/CanConsumer
    ${CMAKE_CURRENT_SOURCE_DIR}/ProbeApp
)

target_link_libraries(ADprobe
    PRIVATE
    ${ARA_LIBRARIES}
    ${ARAExec_LIBRARIES}
    ${ara-com_LIBRARIES}
    ${ara-log_LIBRARIES}
    ${ara-per_LIBRARIES}
    Threads::Threads
)

target_compile_options(ADprobe
    PRIVATE
    -std=c++14
)

install(
    TARGETS ADprobe
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
