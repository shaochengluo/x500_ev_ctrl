#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "microxrcedds_client" for configuration "MinSizeRel"
set_property(TARGET microxrcedds_client APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(microxrcedds_client PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "C"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libmicroxrcedds_client.a"
  )

list(APPEND _cmake_import_check_targets microxrcedds_client )
list(APPEND _cmake_import_check_files_for_microxrcedds_client "${_IMPORT_PREFIX}/lib/libmicroxrcedds_client.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
