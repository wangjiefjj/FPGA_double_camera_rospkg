#include_directories(${PROJECT_SOURCE_DIR}/include)
#link_directories(${PROJECT_SOURCE_DIR}/lib)
find_package(OpenCV REQUIRED)

find_package(catkin  REQUIRED COMPONENTS cv_bridge image_transport  roscpp sensor_msgs std_msgs)

catkin_package()

aux_source_directory(. SRC_LIST)
add_executable(readDoubleCam cybulk_read.cpp)
target_link_libraries(readDoubleCam ${OpenCV_LIBS} libcyusb.so ${catkin_LIBRARIES})
