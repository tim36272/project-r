/*
 * common.cpp
    Copyright (C) 2013  Timothy Sweet

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/console.h>
// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#include "Types.h"
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

// A macro to disallow the default constructor
// This should be used in the private: declarations for a class
#define DISALLOW_DEFAULT_CONSTRUCTION(TypeName) \
  TypeName();

// A macro to set the ROS log level for the active node to debug
#define setLoggerDebug() if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) ros::console::notifyLoggerLevelsChanged();

