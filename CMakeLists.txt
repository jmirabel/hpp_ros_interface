# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
# INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp_ros_interface)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "")
SET(PROJECT_VERSION "")

SET(INSTALL_DOCUMENTATION OFF)
SETUP_PROJECT()
# PROJECT(hpp_ros_interface)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs message_generation)

add_service_files(FILES
    SetJointNames.srv
    SetString.srv
  )

add_message_files(FILES
    PlanningGoal.msg
    ProblemSolved.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(DEPENDS message_runtime)
ADD_OPTIONAL_DEPENDENCY("hpp-corbaserver")
ADD_OPTIONAL_DEPENDENCY("hpp-manipulation-corba")

INSTALL(FILES
  src/hpp_ros/trajectory_publisher.py
  src/hpp_ros/planning_request_adapter.py
  src/hpp_ros/hpp_server_initializer.py
  src/hpp_ros/client.py
  src/hpp_ros/__init__.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

INSTALL(FILES
  src/hpp_ros/manipulation/hpp_server_initializer.py
  src/hpp_ros/manipulation/__init__.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/manipulation)

INSTALL(PROGRAMS scripts/hpp_node.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
INSTALL(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

IF(HPP_CORBASERVER_FOUND)
  INSTALL(PROGRAMS ${HPP_CORBASERVER_PREFIX}/bin/hppcorbaserver
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ENDIF(HPP_CORBASERVER_FOUND)
IF(HPP_MANIPULATION_CORBA_FOUND)
  INSTALL(PROGRAMS ${HPP_MANIPULATION_CORBA_PREFIX}/bin/hpp-manipulation-server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ENDIF(HPP_MANIPULATION_CORBA_FOUND)

SETUP_PROJECT_FINALIZE()