# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseToolsDepthQuality)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

if(${BUILD_EASYLOGGINGPP})
include(../../include/librealsense2/utilities/easylogging/easyloggingpp.cmake)
include_directories( ${ELPP_INCLUDES} )
endif()

include(../../common/CMakeLists.txt)

SET(DELAYED 
    realsense2d.dll
    realsense2-gld.dll
    realsense2.dll
    realsense2-gl.dll
)

if(BUILD_GRAPHICAL_EXAMPLES)
    set(RS_QUALITY_TOOL_CPP
        ${COMMON_SRC}
        rs-depth-quality.cpp
        depth-quality-model.h
        depth-quality-model.cpp
        depth-metrics.h
        ../../common/realsense-ui-advanced-mode.h
        ../../third-party/imgui/imgui.cpp
        ../../third-party/imgui/imgui_draw.cpp
        ../../third-party/imgui/imgui_impl_glfw.cpp
        ../../third-party/imgui/imgui-fonts-karla.hpp
        ../../third-party/imgui/imgui-fonts-fontawesome.hpp
        ../../third-party/glad/glad.c
        ../../third-party/tinyfiledialogs/tinyfiledialogs.c
        ../../third-party/tinyfiledialogs/tinyfiledialogs.h
        )

    if(WIN32)
        add_executable(rs-depth-quality WIN32 ${RS_QUALITY_TOOL_CPP}
                ${ELPP_SOURCES}
                ${CMAKE_CURRENT_SOURCE_DIR}/res/resource.h
                ${CMAKE_CURRENT_SOURCE_DIR}/res/realsense-viewer.rc
                ../../common/windows-app-bootstrap.cpp)

        set_property(TARGET rs-depth-quality PROPERTY CXX_STANDARD 11)
        source_group("Resources" FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/res/resource.h
                ${CMAKE_CURRENT_SOURCE_DIR}/res/realsense-viewer.rc)

        source_group("3rd Party" FILES
            ../../third-party/tinyfiledialogs/tinyfiledialogs.c
            ../../third-party/tinyfiledialogs/tinyfiledialogs.h
            ../../third-party/imgui/imgui.cpp
            ../../third-party/imgui/imgui_draw.cpp
            ../../third-party/imgui/imgui_impl_glfw.cpp
            ../../third-party/imgui/imgui-fonts-karla.hpp
            ../../third-party/imgui/imgui-fonts-fontawesome.hpp)

        source_group("Utils" FILES
            depth-quality-model.h
            depth-quality-model.cpp
            depth-metrics.h
            ${COMMON_SRC})

        include_directories(../../common  ../../third-party
                            ../../third-party/imgui
                            ../../third-party/glad
                            ../../third-party/tclap/include
                            ../../third-party/tinyfiledialogs
                            ${CMAKE_CURRENT_SOURCE_DIR}/res/)

        #list(TRANSFORM DELAYED PREPEND " /DELAYLOAD:")
        PREPEND(DELAYED " /DELAYLOAD:" ${DELAYED})
        #infoValue(DELAYED)
        string(REPLACE ";" " " LD_FLAGS_STR "${DELAYED}")
        set_target_properties(rs-depth-quality PROPERTIES LINK_FLAGS "${LD_FLAGS_STR}")

    else()
        add_executable(rs-depth-quality 
        ${RS_QUALITY_TOOL_CPP}
        ${ELPP_SOURCES}
        )

        include_directories(../../common  ../../third-party
                            ../../third-party/imgui
                            ../../third-party/glad
                            ../../third-party/tinyfiledialogs
                            ../../third-party/tclap/include)
    endif()

    source_group("SW-Update" FILES ${SW_UPDATE_FILES})

    if (CHECK_FOR_UPDATES)
        message( STATUS "Check for updates capability added to realsense-depth-quality" )
        add_dependencies(rs-depth-quality libcurl)
        target_link_libraries(rs-depth-quality curl)
    endif()
    
    target_link_libraries(rs-depth-quality ${DEPENDENCIES}
                                           ${GTK3_LIBRARIES}
                                           Threads::Threads
                                           realsense2-gl)

    if (IMPORT_DEPTH_CAM_FW)
        add_definitions(-DINTERNAL_FW)
        target_link_libraries(rs-depth-quality fw)
    endif()
    
    


    set_target_properties (rs-depth-quality PROPERTIES
        FOLDER Tools
    )

    target_include_directories(rs-depth-quality PRIVATE ../../src)

    install(
        TARGETS

        rs-depth-quality

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}
    )
endif()
