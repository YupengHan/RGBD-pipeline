cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)
project(BallPerception)

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-10.1)
add_definitions(-DCUDA_ON)

#find_package(OpenCV 3 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#list(APPEND renderer_inc ${OpenCV_INCLUDE_DIRS})
#list(APPEND renderer_lib ${OpenCV_LIBS})

find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O3")
list(APPEND renderer_inc ${CUDA_INCLUDE_DIRS})
list(APPEND renderer_lib ${CUDA_LIBRARIES})
list(APPEND renderer_lib ${CUDA_CUBLAS_LIBRARIES})


SET(renderer_cuda src/pl.cu)
SET(renderer_srcs src/main.cpp)

CUDA_COMPILE(renderer_cuda_objs ${renderer_cuda}})

add_library(cuda_renderer
               ${renderer_srcs}
               ${renderer_cuda}
               ${renderer_cuda_objs} 
)

target_include_directories(cuda_renderer PUBLIC ${renderer_inc})
target_link_libraries(cuda_renderer PUBLIC ${renderer_lib})

add_executable(renderer_test src/main.cpp)
