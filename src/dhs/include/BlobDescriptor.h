/*
 * BlobDescriptor.h
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

#ifndef BLOBDESCRIPTOR_H_
#define BLOBDESCRIPTOR_H_
#include <utility>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "common.h"
#include "dhs/contour.h"
#include "Kalman.h"

//describes one connected blob
class BlobDescriptor {
public:
	DISALLOW_COPY_AND_ASSIGN(BlobDescriptor);
	DISALLOW_DEFAULT_CONSTRUCTION(BlobDescriptor);
	BlobDescriptor(int id);
	/*
	 * input_image is a color image with black in background areas
	 */
	void update(int sequence_number, Contour& swapped_contour);
	//serialize the most recent contour
	void serializeContour(ros::Publisher& pub);
	void deserializeContour(int sequence_number, const dhs::contour::_contour_type& contour);

	/*
	 * Accessors
	 */
	//get this blob's unique identifier
	int Id() const;
	//get an arbitrary raw bound
	cv::Rect getRawBound(int index) const;
	//get more recent raw bound
	cv::Rect getLastRawBound() const;
	//get the first timestamp seen
	int firstSeen() const;
	//get the last timestamp seen
	int lastSeen() const;
	//get most recent contour
	const Contour& getLastContour() const;


protected:
	int id_;

	//things that describe a blob
	std::vector<Contour> contours_;
	std::vector<int> sequence_numbers_;
	std::vector<int> depths_;
	std::vector<cv::Rect> raw_bounds_;

};
#endif /* BLOBDESCRIPTOR_H_ */
