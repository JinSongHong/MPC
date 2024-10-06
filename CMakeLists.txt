# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  JinSong
  VERSION 0.1
  DESCRIPTION "Riemannian simulation"
  LANGUAGES CXX)



# file(GLOB_RECURSE SOURCES "ROPTLIB/*.cpp" "main.cpp" "controller.cpp" "globals.cpp" "Mathtool.cpp" "MotionPolicy.cpp" "MyProblem.cpp")
file(GLOB_RECURSE SOURCES "ROPTLIB/*.cpp" "main.cc" "globals.cpp" "filtertool.cpp" "Mathtool.cpp" "MPC.cpp"
"MyProblem.cpp" "Kinematics.cpp" "Controller.cpp" "Trajectory.cpp" "ProdManifold.cpp")
# add_executable (main main.cpp src/foo.cpp src/shape.cpp src/controller.cpp)

# add_executable(main ${SOURCES})
# set(SOURCES
#     main.cc
# )

add_executable (main ${SOURCES})
# add_executable (main)
# add_executable (main ${SRC_FILES})




target_include_directories(main PUBLIC
${CMAKE_SOURCE_DIR}/include
${CMAKE_SOURCE_DIR}/../../include
/usr/include
/home/jinsong/casadi/build/lib
/home/jinsong/casadi/build
/home/jinsong/casadi
/home/jinsong/lib/Ipopt/src/Interfaces
/home/jinsong/lib/Ipopt/src/Common
/home/jinsong/lib/Ipopt/src/LinAlg
/usr/include/eigen-master/unsupported/Eigen
${CMAKE_SOURCE_DIR}/ROPTLIB
${CMAKE_SOURCE_DIR}/ROPTLIB/cwrapper/lapack
${CMAKE_SOURCE_DIR}/ROPTLIB/cwrapper/blas
${CMAKE_SOURCE_DIR}/ROPTLIB/test
${CMAKE_SOURCE_DIR}/ROPTLIB/Solvers
${CMAKE_SOURCE_DIR}/ROPTLIB/Problems
${CMAKE_SOURCE_DIR}/ROPTLIB/Others/wavelet
${CMAKE_SOURCE_DIR}/ROPTLIB/Others/fftw
${CMAKE_SOURCE_DIR}/ROPTLIB/Others/SparseBLAS
${CMAKE_SOURCE_DIR}/ROPTLIB/Others
${CMAKE_SOURCE_DIR}/ROPTLIB/Manifolds
  )

target_link_directories(main PUBLIC           #Library directory
    ${CMAKE_SOURCE_DIR}/../../lib
    
)

target_link_libraries(main
    lapack
    blas
    casadi
    ipopt
    mujoco
    glfw
    
)