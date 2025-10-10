# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr")
  file(MAKE_DIRECTORY "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr")
endif()
file(MAKE_DIRECTORY
  "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build"
  "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install"
  "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp"
  "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
  "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src"
  "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp${cfgdir}") # cfgdir has leading slash
endif()
