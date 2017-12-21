cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(VS_Practical_2a)

find_package(OpenCV REQUIRED)
include_directories("/usr/include/libusb-1.0/")

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
	INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

ADD_EXECUTABLE(3DmodelBasedTracking
MyFreenectDevice.hpp
main.cpp)

target_link_libraries(3DmodelBasedTracking ${OpenCV_LIBS})
