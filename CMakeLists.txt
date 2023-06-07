#Drake Hello World Test
#Egwuchukwu Kalu
cmake_minimum_required(VERSION 3.10)
include_directories(/usr/local/include/mujoco /usr/local/lib )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/eakorl/Documents/Engineering/Research/Code/include)


project(Planar_2R_Operational_Space_Control)
find_package(drake REQUIRED)
find_package(mujoco REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
set(CMAKE_BUILD_TYPE Debug)

add_executable(OSC_QP include/Quadratic_Programming_OSC.cc)
target_link_libraries(OSC_QP drake::drake mujoco::mujoco GLEW::GLEW)

#add_executable(File_Stuff include/file_manage.cc)
