/*
 * BlobDescriptor.cpp
    Copyright (C) 2013 Timothy Sweet

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
#include "dhs/BlobDescriptor.h"
#include "dhs/Utility.h"
#include <stdexcept>

BlobDescriptor::BlobDescriptor(int id) {
	id_ = id;
}

void BlobDescriptor::update(int sequence_number, Contour& swapped_contour) {

	//store the contour
	contours_.push_back(Contour());
	swapped_contour.swap(*contours_.rbegin());

	//store sequence number
	sequence_numbers_.push_back(sequence_number);

	//store raw bound
	raw_bounds_.push_back(cv::boundingRect(*contours_.rbegin()));

	//call child updater
	update_child();
}

void BlobDescriptor::serializeBlob(ros::Publisher& pub) {
	dhs::blobPtr msg(new dhs::blob());
	msg->id = id_;
	//for each point in the contour, put it in the blob
	ContourConstIt cursor = contours_.rbegin()->begin();
	for(;cursor!=contours_.rbegin()->end();cursor++) {
		msg->contour.push_back(cursor->x);
		msg->contour.push_back(cursor->y);
	}
	msg->raw_position[0] = getLastRawBound().x;
	msg->raw_position[1] = getLastRawBound().y;
	msg->raw_size[0] = getLastRawBound().width;
	msg->raw_size[1] = getLastRawBound().height;
//	msg->depth = getLastDepth();
	//serialize any data from a decorated class
	serialize_decorators(msg);
	pub.publish(msg);
}

void BlobDescriptor::deserializeBlob(const dhs::blobPtr& blob) {
	Contour out;
	//for each point in the contour, put it in the blob
	dhs::blob::_contour_type::const_iterator cursor = blob->contour.begin();
	assert(blob->contour.size()%2==0);
	for(;cursor!=blob->contour.end();) {
		cv::Point pt(*(cursor++),*(cursor++));
		int temp = pt.x;
		pt.x = pt.y;
		pt.y=temp;
		out.push_back(pt);
	}
	raw_bounds_.push_back(cv::Rect(blob->raw_position[0],
								   blob->raw_position[1],
								   blob->raw_size[0],
								   blob->raw_size[1]));
//	depths_.push_back(blob.depth);
	deserialize_decorators(blob);
	this->update(blob->header.seq,out);
}

int BlobDescriptor::Id() const {
	return id_;
}

cv::Rect BlobDescriptor::getLastRawBound() const {
	assert(!raw_bounds_.empty());
	return *raw_bounds_.rbegin();
}

cv::Rect BlobDescriptor::getRawBound(int index) const {
	try {
		return raw_bounds_.at(index);
	} catch (const std::out_of_range& oor){
		std::cout<<"requested out of range bound index"<<std::endl;
		assert(false);
	}
	//dummy return to make the compiler happy
	return (cv::Rect());
}



int BlobDescriptor::firstSeen() const{
	assert(sequence_numbers_.size() > 0);
	return *sequence_numbers_.begin();
}

int BlobDescriptor::lastSeen() const{
	assert(sequence_numbers_.size() > 0);
	return *sequence_numbers_.rbegin();
}

const Contour& BlobDescriptor::getLastContour() const{
	assert(!contours_.empty());
	return *contours_.rbegin();
}


int BlobDescriptor::getLastDepth() const {
	assert(!depths_.empty());
	return *depths_.rbegin();
}

int BlobDescriptor::getDepth(int index) const {
	try {
		return depths_.at(index);
	} catch (const std::out_of_range& oor){
		std::cout<<"requested out of range depth index"<<std::endl;
		assert(false);
	}
	//dummy return to make the compiler happy
	return -1;
}
