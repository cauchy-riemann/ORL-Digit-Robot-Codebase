#Drake Hello World Test
#Egwuchukwu Kalu
cmake_minimum_required(VERSION 3.10)
include_directories(/usr/local/include/mujoco /usr/local/lib )
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/kalu/Documents/Files/Engineering/Research/Code/Main_Workspace)


project(Planar_2R_Operational_Space_Control)
find_package(mujoco REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(qpOASES REQUIRED)
set(CMAKE_BUILD_TYPE Debug)

add_executable(OSC_QP include/P2R_OSC.cc)
target_link_libraries(OSC_QP mujoco::mujoco GLEW::GLEW qpOASES)

#add_executable(File_Stuff include/file_manage.cc)
