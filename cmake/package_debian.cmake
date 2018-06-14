# general cpack variables
set(CPACK_PACKAGE_CONTACT "Roboception <info@roboception.de>")
set(CPACK_PACKAGE_VENDOR "Roboception GmbH, Munich, Germany")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Roboception ${PROJECT_NAME} package")

###############################
# debian package specific stuff
###############################
set(CPACK_GENERATOR "DEB")
#set(CPACK_DEBIAN_PACKAGE_DEBUG ON)

if (NOT CPACK_DEBIAN_PACKAGE_ARCHITECTURE)
    find_program(DPKG_CMD dpkg)
    mark_as_advanced(DPKG_CMD)
    if (NOT DPKG_CMD)
        message(STATUS "Can not find dpkg in your path, default to i386.")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
    else ()
        execute_process(COMMAND "${DPKG_CMD}" --print-architecture
                OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
                OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif ()
endif ()

# package name is lower case of project name with _ replaced by -
string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWER)
string(REPLACE "_" "-" CPACK_PACKAGE_NAME "${PROJECT_NAME_LOWER}")

# check if it is a ROS/catkin package
if (EXISTS "${PROJECT_SOURCE_DIR}/package.xml")
    file(STRINGS "${PROJECT_SOURCE_DIR}/package.xml" PACKAGE_XML_VERSION REGEX <version>[0-9.]*</version>)
    string(REGEX REPLACE .*<version>\([0-9.]*\)</version>.* \\1 ROS_PACKAGE_VERSION "${PACKAGE_XML_VERSION}")
    if (NOT ROS_PACKAGE_VERSION MATCHES ${PROJECT_VERSION})
        message(WARNING "Version in package.xml (${ROS_PACKAGE_VERSION}) doesn't match project version (${PROJECT_VERSION})")
    endif ()
    set(ROS_DISTRO $ENV{ROS_DISTRO})
    if (NOT ROS_DISTRO)
        message(WARNING "ROS_DISTRO not set! falling back to indigo")
        set(ROS_DISTRO "indigo")
    endif ()
    set(CPACK_PACKAGE_NAME "ros-${ROS_DISTRO}-${CPACK_PACKAGE_NAME}")

    # tell CPack to use CMAKE_INSTALL_PREFIX
    # cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/indigo" -DCMAKE_PREFIX_PATH="/opt/ros/indigo" -DCMAKE_BUILD_TYPE=Release ..
    set(CPACK_SET_DESTDIR true)
endif ()

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

#########################################
## things you might need to change ??? ##
#########################################
# ??? comment this line if this package doesn't provide any libs or binaries
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

# ??? this defaults to PROJECT_LIBRARIES which should be defined in main CMakeLists.txt before including this file
# list of shared library this package provides (; separated, comment or empty list if there are none)
# IMPORTANT: also the lib needs to set SOVERSION via set_target_properties, e.g.:
# set_target_properties(rcimage PROPERTIES SOVERSION ${abiversion})
if (PROJECT_LIBRARIES)
    set(sharedlibs ${PROJECT_LIBRARIES})
endif ()

# if there are shared libs exported by this package:
# generate debian shlibs file and call ldconf in postinst and postrm scripts
if (sharedlibs)
    set(SHLIBS_FILE "${CMAKE_CURRENT_BINARY_DIR}/shlibs")
    set(POSTINST_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/postinst")
    set(POSTRM_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/postrm")

    # Generate postinst, prerm and postrm hooks
    file(WRITE "${POSTINST_SCRIPT}" "#!/bin/sh\n\nset -e\n")
    file(WRITE "${POSTRM_SCRIPT}" "#!/bin/sh\n\nset -e\n")
    file(APPEND "${POSTINST_SCRIPT}" "if [ \"$1\" = \"configure\" ]; then\n        ldconfig\nfi\n")
    file(APPEND "${POSTRM_SCRIPT}" "if [ \"$1\" = \"remove\" ]; then\n        ldconfig\nfi\n")

    # Generate shlibs file
    # also the lib needs to set SOVERSION via set_target_properties:
    # set_target_properties(rcimage PROPERTIES SOVERSION ${abiversion})
    set(abiversion "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
    file(WRITE "${SHLIBS_FILE}" "")
    foreach (libname ${sharedlibs})
        file(APPEND "${SHLIBS_FILE}" "${libname} ${abiversion} ${CPACK_PACKAGE_NAME}\n")
    endforeach (libname)

    execute_process(COMMAND chmod 644 "${SHLIBS_FILE}")
    execute_process(COMMAND chmod 755 "${POSTINST_SCRIPT}" "${POSTRM_SCRIPT}")
    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${SHLIBS_FILE};${POSTINST_SCRIPT};${POSTRM_SCRIPT}")
endif ()

include(CPack)
