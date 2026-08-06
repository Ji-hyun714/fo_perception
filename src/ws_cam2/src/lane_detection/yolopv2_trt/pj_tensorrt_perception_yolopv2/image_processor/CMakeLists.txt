cmake_minimum_required(VERSION 3.0)

set(LibraryName "ImageProcessor")

# Create library
add_library (${LibraryName} image_processor.cpp image_processor.h detection_engine.cpp detection_engine.h)

# For OpenCV
find_package(OpenCV REQUIRED)
target_include_directories(${LibraryName} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${LibraryName} ${OpenCV_LIBS})

# For OpenMP (used in detection_engine post-process loop)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${LibraryName} OpenMP::OpenMP_CXX)
endif()

# Link Common Helper module
if(NOT TARGET CommonHelper)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../common_helper common_helper)
endif()
target_include_directories(${LibraryName} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../../common_helper)
target_link_libraries(${LibraryName} CommonHelper)

# For InferenceHelper
set(INFERENCE_HELPER_DIR ${CMAKE_CURRENT_LIST_DIR}/../../InferenceHelper/)
set(INFERENCE_HELPER_ENABLE_TENSORRT ON CACHE BOOL "TENSORRT")
if(NOT TARGET InferenceHelper)
    add_subdirectory(${INFERENCE_HELPER_DIR}/inference_helper inference_helper)
endif()
target_include_directories(${LibraryName} PUBLIC ${INFERENCE_HELPER_DIR}/inference_helper)
target_link_libraries(${LibraryName} InferenceHelper)
