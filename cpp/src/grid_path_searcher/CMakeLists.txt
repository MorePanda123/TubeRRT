cmake_minimum_required(VERSION 2.8.3)
project(grid_path_searcher)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs 
    tf
    kdtree
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# add your code here: find_package(xxx REQUIRED)
# find_package(ompl REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

link_libraries(${OMPL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror


add_executable(tube_path_plan
    src/tubeRRTStar.cpp
    src/graph_searcher.cpp
    src/corridor_finder_mao.cpp)

target_link_libraries(tube_path_plan
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${OMPL_LIBRARIES}
)


add_executable ( random_complex 
    src/random_complex_generator.cpp )

target_link_libraries( random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  
