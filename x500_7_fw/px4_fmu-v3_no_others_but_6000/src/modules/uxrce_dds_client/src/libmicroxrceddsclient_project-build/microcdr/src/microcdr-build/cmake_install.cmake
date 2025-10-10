# Install script for directory: /home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install/microcdr-2.0.1")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/arm-none-eabi-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/libmicrocdr.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ucdr" TYPE DIRECTORY FILES "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr/include/ucdr/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ucdr" TYPE FILE FILES "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/include/ucdr/config.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake/microcdrTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake/microcdrTargets.cmake"
         "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/CMakeFiles/Export/7dbace54a6f331dd3fb26f3c05b88776/microcdrTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake/microcdrTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake/microcdrTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake" TYPE FILE FILES "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/CMakeFiles/Export/7dbace54a6f331dd3fb26f3c05b88776/microcdrTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake" TYPE FILE FILES "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/CMakeFiles/Export/7dbace54a6f331dd3fb26f3c05b88776/microcdrTargets-minsizerel.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/microcdr/cmake" TYPE FILE FILES
    "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/cmake/config/microcdrConfig.cmake"
    "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/cmake/config/microcdrConfigVersion.cmake"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/cpsl/PX4-Autopilot_1.15.4/build/px4_fmu-v3_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
