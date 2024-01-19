

unset(GLOG_FOUND)

if (NOT DEFINED GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION
    AND NOT GLOG_INCLUDE_DIR_HINTS
    AND NOT GLOG_LIBRARY_DIR_HINTS)
  glog_message(STATUS "No preference for use of exported glog CMake "
    "configuration set, and no hints for include/library directories provided. "
    "Defaulting to preferring an installed/exported glog CMake configuration "
    "if available.")
  set(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION TRUE)
endif()


if (GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION)
  find_package(glog QUIET
                    NAMES google-glog glog
                    HINTS ${glog_DIR} ${HOMEBREW_INSTALL_PREFIX}
                    NO_MODULE
                    NO_CMAKE_PACKAGE_REGISTRY
                    NO_CMAKE_BUILDS_PATH)
  if (glog_FOUND)
    glog_message(STATUS "Found installed version of glog: ${glog_DIR}")
  else()
    # Failed to find an installed version of glog, repeat search allowing
    # exported build directories.
    glog_message(STATUS "Failed to find installed glog CMake configuration, "
      "searching for glog build directories exported with CMake.")
    # Again pass NO_CMAKE_BUILDS_PATH, as we know that glog is exported and
    # do not want to treat projects built with the CMake GUI preferentially.
    find_package(glog QUIET
                      NAMES google-glog glog
                      NO_MODULE
                      NO_CMAKE_BUILDS_PATH)
    if (glog_FOUND)
      glog_message(STATUS "Found exported glog build directory: ${glog_DIR}")
    endif(glog_FOUND)
  endif(glog_FOUND)

  set(FOUND_INSTALLED_GLOG_CMAKE_CONFIGURATION ${glog_FOUND})

  if (FOUND_INSTALLED_GLOG_CMAKE_CONFIGURATION)
    glog_message(STATUS "Detected glog version: ${glog_VERSION}")
    set(GLOG_FOUND ${glog_FOUND})
    # glog wraps the include directories into the exported glog::glog target.
    set(GLOG_INCLUDE_DIR "")
    set(GLOG_LIBRARY glog::glog)
  else (FOUND_INSTALLED_GLOG_CMAKE_CONFIGURATION)
    glog_message(STATUS "Failed to find an installed/exported CMake "
      "configuration for glog, will perform search for installed glog "
      "components.")
  endif (FOUND_INSTALLED_GLOG_CMAKE_CONFIGURATION)
endif(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION)

if (NOT GLOG_FOUND)

  # Search user-installed locations first, so that we prefer user installs
  # to system installs where both exist.
  list(APPEND GLOG_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
  # Windows (for C:/Program Files prefix).
  list(APPEND GLOG_CHECK_PATH_SUFFIXES
    glog/include
    glog/Include
    Glog/include
    Glog/Include
    google-glog/include # CMake installs with project name prefix.
    google-glog/Include)

  list(APPEND GLOG_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib)
  # Windows (for C:/Program Files prefix).
  list(APPEND GLOG_CHECK_LIBRARY_SUFFIXES
    glog/lib
    glog/Lib
    Glog/lib
    Glog/Lib
    google-glog/lib # CMake installs with project name prefix.
    google-glog/Lib)

  # Search supplied hint directories first if supplied.
  find_path(GLOG_INCLUDE_DIR
    NAMES glog/logging.h
    HINTS ${GLOG_INCLUDE_DIR_HINTS}
    PATHS ${GLOG_CHECK_INCLUDE_DIRS}
    PATH_SUFFIXES ${GLOG_CHECK_PATH_SUFFIXES})
  if (NOT GLOG_INCLUDE_DIR OR
      NOT EXISTS ${GLOG_INCLUDE_DIR})
    glog_report_not_found(
      "Could not find glog include directory, set GLOG_INCLUDE_DIR "
      "to directory containing glog/logging.h")
  endif (NOT GLOG_INCLUDE_DIR OR
    NOT EXISTS ${GLOG_INCLUDE_DIR})

  find_library(GLOG_LIBRARY NAMES glog
    HINTS ${GLOG_LIBRARY_DIR_HINTS}
    PATHS ${GLOG_CHECK_LIBRARY_DIRS}
    PATH_SUFFIXES ${GLOG_CHECK_LIBRARY_SUFFIXES})
  if (NOT GLOG_LIBRARY OR
      NOT EXISTS ${GLOG_LIBRARY})
    glog_report_not_found(
      "Could not find glog library, set GLOG_LIBRARY "
      "to full path to libglog.")
  endif (NOT GLOG_LIBRARY OR
    NOT EXISTS ${GLOG_LIBRARY})

  # Mark internally as found, then verify. GLOG_REPORT_NOT_FOUND() unsets
  # if called.
  set(GLOG_FOUND TRUE)

  if (GLOG_INCLUDE_DIR AND
      NOT EXISTS ${GLOG_INCLUDE_DIR}/glog/logging.h)
    glog_report_not_found(
      "Caller defined GLOG_INCLUDE_DIR:"
      " ${GLOG_INCLUDE_DIR} does not contain glog/logging.h header.")
  endif (GLOG_INCLUDE_DIR AND
    NOT EXISTS ${GLOG_INCLUDE_DIR}/glog/logging.h)
  # TODO: This regex for glog library is pretty primitive, we use lowercase
  #       for comparison to handle Windows using CamelCase library names, could
  #       this check be better?
  string(TOLOWER "${GLOG_LIBRARY}" LOWERCASE_GLOG_LIBRARY)
  if (GLOG_LIBRARY AND
      NOT "${LOWERCASE_GLOG_LIBRARY}" MATCHES ".*glog[^/]*")
    glog_report_not_found(
      "Caller defined GLOG_LIBRARY: "
      "${GLOG_LIBRARY} does not match glog.")
  endif (GLOG_LIBRARY AND
    NOT "${LOWERCASE_GLOG_LIBRARY}" MATCHES ".*glog[^/]*")

  glog_reset_find_library_prefix()

endif(NOT GLOG_FOUND)

# Set standard CMake FindPackage variables if found.
if (GLOG_FOUND)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
endif (GLOG_FOUND)

if (GLOG_FOUND)
  mark_as_advanced(FORCE GLOG_INCLUDE_DIR
                         GLOG_LIBRARY
                         glog_DIR) # Autogenerated by find_package(glog)
endif (GLOG_FOUND)
