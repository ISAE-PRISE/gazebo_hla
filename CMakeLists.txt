# ----------------------------------------------------------------------------
# Gazebo HLA - HLA Gazebo Plugin for Run-Time Infrastructure (RTI)
# Copyright (C) 2018  ISAE
#
# This program is free software ; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation ; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY ; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program ; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# Author: 
# 	Jean-Baptiste Chaudron
#	jean-baptiste.chaudron@isae-supaero.fr
#
# Version: 
#	1.0.0
# 
# RTI:
#	Tested with CERTI
#	https://savannah.nongnu.org/projects/certi
# 
#
PROJECT(Gazebo-HLA-PROJECT)

#################### CERTI requires CMake 2.8.11 version ###########################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
#SET(CMAKE_CXX_FLAGS "-std=c++11")

CMAKE_POLICY(SET CMP0014 OLD)
cmake_policy(SET CMP0003 NEW)

IF ("$ENV{CERTI_HOME}" STRGREATER "")
    FILE(TO_CMAKE_PATH "$ENV{CERTI_HOME}" CERTI_HOME_TEMP)
    SET(CERTI_HOME ${CERTI_HOME_TEMP} CACHE PATH "The CERTI install directory")
    LIST(APPEND CMAKE_MODULE_PATH "${CERTI_HOME}/share/scripts")

ELSE ("$ENV{CERTI_HOME}" STRGREATER "")
    LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/scripts")    
ENDIF ("$ENV{CERTI_HOME}" STRGREATER "")

OPTION(USE_FULL_RPATH
             "Use the full RPATH" OFF)
             
IF (USE_FULL_RPATH)       
    # see http://www.cmake.org/Wiki/CMake_RPATH_handling#Always_full_RPATH
    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
    
    # when building, don't use the install RPATH already
    # (but later on when installing)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    
    # the RPATH to be used when installing
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    
    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
ENDIF(USE_FULL_RPATH)


OPTION(BUILD_SHARED
  "Build libraries as shared library" OFF)
IF (BUILD_SHARED)
  SET(BUILD_SHARED_LIBS ON)
ELSE (BUILD_SHARED)
  SET(BUILD_SHARED_LIBS OFF)
ENDIF (BUILD_SHARED)

#SET(CMAKE_BUILD_TYPE "Debug")
###
SET(CMAKE_BUILD_TYPE "Release")

add_definitions(-DRTI_USES_STD_FSTREAM)

SET(PACKAGE_NAME      ${CMAKE_PROJECT_NAME})
SET(PACKAGE_VERSION_MAJOR "0")
SET(PACKAGE_VERSION_MINOR "0")
SET(PACKAGE_VERSION_PATCH "1")
SET(PACKAGE_VERSION   "${PACKAGE_VERSION_MAJOR}.${PACKAGE_VERSION_MINOR}.${PACKAGE_VERSION_PATCH}")
###
SET(PACKAGE_BUGREPORT "jean-baptiste.chaudron@isae-supaero.fr")

MESSAGE(STATUS "Configuring ${PACKAGE_NAME} version ${PACKAGE_VERSION}")

# CERTI
FIND_PACKAGE(RTI REQUIRED)
INCLUDE_DIRECTORIES(${RTI_INCLUDE_DIR})
ADD_DEFINITIONS(${RTI_DEFINITIONS})
include_directories(${CERTI_INCLUDE_DIRS}/hla13 ${CERTI_INCLUDE_DIRS}/libhla)
MESSAGE(STATUS "* CERTI FOUND *")

# GAZEBO
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
MESSAGE(STATUS "* GAZEBO FOUND *")

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Gazebo HLA using CERTI (for now)")

SET(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
SET(CPACK_SYSTEM_NAME ${CMAKE_SYSTEM_NAME})

IF ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
     SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\\\.svn/;\\\\.swp$;\\\\.#;/#;.*~;cscope.*;\\\\.cproject;\\\\.project;autom4te.cache/*")
ELSE ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
     SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\\\.svn/;\\\\.swp$;\\\\.#;/#;.*~;cscope.*;\\\\.cproject;\\\\.project;autom4te.cache/*;${CMAKE_BINARY_DIR}/*")
ENDIF ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")

SET(CPACK_SOURCE_GENERATOR "ZIP;TGZ")
IF (WIN32)
  SET(CPACK_GENERATOR "ZIP;NSIS")
  GET_FILENAME_COMPONENT(CPACK_COMMAND ${CMAKE_COMMAND} PATH)
  SET(CPACK_COMMAND ${CPACK_COMMAND}/cpack)
  ADD_CUSTOM_TARGET(PACKAGE_SOURCE_ZIP
              COMMAND ${CPACK_COMMAND} -G ZIP --config CPackSourceConfig.cmake
              COMMENTS "Build a ZIP file containing the source")  
  INCLUDE(InstallRequiredSystemLibraries)
ELSE (WIN32)
  SET(CPACK_GENERATOR "ZIP;TGZ")
ENDIF(WIN32)
SET(CPACK_PACKAGE_VERSION ${PACKAGE_VERSION})
SET(CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION_PATCH})

INCLUDE(CPack)

# FOR Gazebo_FED 
SET(Gazebo_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)
SET(Gazebo_FED_INCLUDE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)
ADD_SUBDIRECTORY(${Gazebo_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${Gazebo_FED_INCLUDE_DIRECTORY})

SET(Gazebo_MAIN_FILE_HLA13 ${Gazebo_FED_SOURCE_DIRECTORY}/GazeboModelPluginHla13.cc)

# FOR FOM FILES 
SET(FOM_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/fom)
ADD_SUBDIRECTORY(${FOM_DIRECTORY})

# FOR CONFIGS FILES 
SET(CONFIGS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/configs)
ADD_SUBDIRECTORY(${CONFIGS_DIRECTORY})


###########   the executable name & the source list  ############################
## HLA 1.3 specific code follows
set(FEDERATE_TARGETNAME "GazeboFederateHla13")
add_library(${FEDERATE_TARGETNAME} SHARED 
               ${Gazebo_MAIN_FILE_HLA13} 
               ${Gazebo_FED_SOURCE_DIRECTORY}/GazeboFederateHla13.cc 
               ${Gazebo_FED_SOURCE_DIRECTORY}/MessageBuffer.cc 
               )
          
target_link_libraries(${FEDERATE_TARGETNAME} ${RTI_LIBRARIES} ${GAZEBO_LIBRARIES})
INSTALL(TARGETS ${FEDERATE_TARGETNAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${CONFIGS_DIRECTORY}/GazeboFederateHla13.world DESTINATION lib)
INSTALL(FILES ${FOM_DIRECTORY}/ASNVA.fed DESTINATION lib)


MESSAGE(STATUS "************************************************************************")
MESSAGE(STATUS "**********                                                    **********")
MESSAGE(STATUS "********** ${CMAKE_PROJECT_NAME} has been successfully configured **********")
MESSAGE(STATUS "**********                                                    **********")
MESSAGE(STATUS "************************************************************************")
