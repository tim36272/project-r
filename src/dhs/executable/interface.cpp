/*
 * interface.cpp
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

#include <ros/ros.h>

#include "../include/ImageFetcher.h"
#include "../include/BlobDescriptor.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "image_filter");
	ros::NodeHandle handle;
	//get the blobs
	ImageFetcher rgb_stream("rgb_in");
	//subscribe to blob descriptor
	BlobDescriptorFetcher blob_stream("blob_in");
	ros::spin();
}
