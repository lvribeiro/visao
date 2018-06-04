cmake_minimum_required(VERSION 2.8.3)
project(visao)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

catkin_package()

find_package(OpenCV)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_sub src/image_subscriber.cpp)
add_executable(video_pub src/video_publisher.cpp)
add_executable(image_sub_mono src/image_subscriber_mono.cpp)


target_link_libraries(image_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(video_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(image_sub_mono ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})