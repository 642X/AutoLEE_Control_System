#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pinocchio::pinocchio" for configuration "Release"
set_property(TARGET pinocchio::pinocchio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pinocchio::pinocchio PROPERTIES
  IMPORTED_LOCATION_RELEASE "/opt/openrobots/lib/libpinocchio.so.2.7.0"
  IMPORTED_SONAME_RELEASE "libpinocchio.so.2.7.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS pinocchio::pinocchio )
list(APPEND _IMPORT_CHECK_FILES_FOR_pinocchio::pinocchio "/opt/openrobots/lib/libpinocchio.so.2.7.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
