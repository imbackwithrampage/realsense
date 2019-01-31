######## INCLUDE DIRECTORIES ########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

######## PACKAGE DEPENDENCIES ########
######## LIBRARY DEPENDENCIES ########

link_directories(
  ${CMAKE_SOURCE_DIR}/lib
)

set(TEST_LIBRARIES
  gmock_main
  gmock
  gtest
  pthread
)

set(TEST_MAIN_LIB
  rostest_gmock_main
)

set(PIXEL_CONVERSION_SERVICE_TEST_LINK_LIBRARIES
  ${LINK_LIBRARIES}
  ${PROJECT_NAME}
  ${NODELET_NAME}
  ${TEST_MAIN_LIB}
  ${TEST_LIBRARIES}
  ${catkin_LIBRARIES}
)

######## SOURCES ########

set(PIXEL_CONVERSION_SERVICE_TEST_SOURCE src/service.cc)

######## BUILD ##########

set(PIXEL_CONVERSION_SERVICE_TEST_NAME realsense_pixel_conversion_service_test)

add_rostest_gmock (${PIXEL_CONVERSION_SERVICE_TEST_NAME} src/service.test ${PIXEL_CONVERSION_SERVICE_TEST_SOURCE})

target_link_libraries(${PIXEL_CONVERSION_SERVICE_TEST_NAME} ${PIXEL_CONVERSION_SERVICE_TEST_LINK_LIBRARIES})


