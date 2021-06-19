cmake_minimum_required(VERSION 3.17)
project(CAL-Proj)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Build GraphViewerCpp
set(GRAPHVIEWERCPP_PATH lib/GraphViewerCpp)
add_subdirectory(${GRAPHVIEWERCPP_PATH})
include_directories(${GRAPHVIEWERCPP_PATH}/include)

file(GLOB CAL_PROJ_FILES CONFIGURE_DEPENDS
        "src/*.cpp"
        )

add_executable(src
        ${CAL_PROJ_FILES}
        src/graph/Edge.h src/loaders/DataLoader.cpp src/loaders/DataLoader.h src/Application.cpp src/Application.h src/users/Baker.cpp src/users/Client.cpp src/utils/utils.cpp src/utils/utils.h src/loaders/GraphLoader.h src/loaders/GraphLoader.cpp src/utils/Csvfile.h)

target_link_libraries(src graphviewer)
