cmake_minimum_required(VERSION 3.5)

project(ViconMavlink VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets Network Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets Network Concurrent)
find_package(vicon-datastream-sdk)

set(PROJECT_SOURCES
    main.cpp
    Drone.cpp
    Sender.cpp
    SenderWindow.cpp
    Station.cpp
    StationController.cpp
    StationWindow.cpp
    SenderController.cpp
    DroneState.h
    Drone.h
    Sender.h
    SenderWindow.h
    Station.h
    StationController.h
    StationWindow.h
    SenderController.h
    StationWindow.ui
    SenderWindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ViconMavlink
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET ViconMavlink APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(ViconMavlink SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(ViconMavlink
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(ViconMavlink PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::Concurrent
    vicon-datastream-sdk::ViconDataStreamSDK_CPP
    wsock32
    Wlanapi
    WS2_32
)

target_compile_definitions(ViconMavlink PRIVATE "_BUILD_STATIC_LIB")
target_compile_options(ViconMavlink PRIVATE -Wno-address-of-packed-member)

set_target_properties(ViconMavlink PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS ViconMavlink
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ViconMavlink)
endif()
