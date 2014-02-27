/*
 * ProcessorNode.h
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

#ifndef PROCESSORNODE_H_
#define PROCESSORNODE_H_
#include <string>
#include <ros/ros.h>
#include "dhs/event.h"
#include "common.h"

//This class is meant to be inherited by other classes
class ProcessorNode {
	public:
	ProcessorNode() {}
	virtual ~ProcessorNode() {};

	protected:
	std::string event_name_;
	std::string output_topic_;
	int event_code_;
	virtual void init() = 0;
	void setupOutput();

	private:
	ros::Publisher output_stream_;
	ros::NodeHandle handle_;


	DISALLOW_COPY_AND_ASSIGN(ProcessorNode);
};

#endif /* PROCESSORNODE_H_ */
