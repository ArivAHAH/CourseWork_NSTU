cmake_minimum_required(VERSION 3.16)
project(traffic-sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS
    Core
    Widgets
    Sql
    WebEngineWidgets
)

include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)

include_directories(/opt/homebrew/include)

add_executable(traffic-sim
    main.cpp
    mainwindow.cpp
    simulation_backend.cpp
    traffic_core.cpp
    intro_widget.cpp
    resources.qrc    
    webbridge.cpp
    simulation_view.cpp  
    build_mode_view.cpp    
)

target_link_libraries(traffic-sim
    Qt6::Core
    Qt6::Widgets
    Qt6::Sql
    Qt6::WebEngineWidgets
    box2d
)
