# Unit tests:

find_package(mola_test_datasets)
find_package(mola_input_rosbag2)

if((NOT mola_test_datasets_FOUND) OR
   (NOT mola_input_rosbag2_FOUND) OR
  )
  message(STATUS "**********************************************************************")
  message(STATUS " WARNING: Skipping unit tests since test dependencies were not found  ")
  message(STATUS "**********************************************************************")

  # If we are in a ROS build farm, unit tests are MANDATORY, not like in user local builds:
  if (("${BUILD_TESTING}" STREQUAL "1") AND ("$ENV{HOME}" STREQUAL "/home/buildfarm"))
    message(SEND_ERROR "** Requirements for unit tests not found (see messages above). Stopping generation since unit tests are required in build farms **")
  endif()

  return()
endif()

