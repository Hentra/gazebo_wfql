cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package (Qt5Core REQUIRED)
find_package (Qt5Widgets REQUIRED)
set (CMAKE_AUTOMOC ON)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library( fido SHARED IMPORTED )
set_target_properties( fido PROPERTIES IMPORTED_LOCATION /usr/local/lib/libfido.so )

add_library(model_wfqi SHARED model_wfqi.cc)
target_link_libraries(model_wfqi ${GAZEBO_LIBRARIES} Qt5::Core fido)

add_library(flag_locator SHARED flag_locator.cc)
target_link_libraries(flag_locator ${GAZEBO_LIBRARIES})
