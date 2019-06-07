find_path(Phidget22_INCLUDE_DIR NAMES phidget22.h)
find_library(Phidget22_LIBRARY NAMES phidget22)

set(libphidget22_VERSION 2.2.0)

mark_as_advanced(Phidget22_FOUND Phidget22_INCLUDE_DIR Phidget22_LIBRARY Phidget22_VERSION)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Phidget22
    REQUIRED_VARS Phidget22_INCLUDE_DIR Phidget22_LIBRARY
    VERSION_VAR Phidget22_VERSION
)

if(Phidget22_FOUND)
    set(Phidget22_INCLUDE_DIRS ${Phidget22_INCLUDE_DIR})
endif()

if(Phidget22_FOUND AND NOT TARGET Phidget22::Phidget22)
    add_library(Phidget22::Phidget22 INTERFACE IMPORTED)
    set_target_properties(Phidget22::Phidget22 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Phidget22_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${Phidget22_LIBRARY}"
    )
endif()
