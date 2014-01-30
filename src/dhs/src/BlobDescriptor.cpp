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
#include "BlobDescriptor.h"
#include "Utility.h"
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
}

void BlobDescriptor::serializeContour(ros::Publisher& pub) {
	dhs::contourPtr msg(new dhs::contour());
	msg->id = id_;
	//for each point in the contour, put it in the blob
	ContourConstIt cursor = contours_.rbegin()->begin();
	for(;cursor!=contours_.rbegin()->end();cursor++) {
		msg->contour.push_back(cursor->x);
		msg->contour.push_back(cursor->y);
	}
	pub.publish(msg);
}

void BlobDescriptor::deserializeContour(int sequence_number, const dhs::contour::_contour_type& contour) {
	Contour out;
	//for each point in the contour, put it in the blob
	dhs::contour::_contour_type::const_iterator cursor = contour.begin();
	assert(contour.size()%2==0);
	for(;cursor!=contour.end();) {
		cv::Point pt(*(cursor++),*(cursor++));
		int temp = pt.x;
		pt.x = pt.y;
		pt.y=temp;
		out.push_back(pt);
	}

	this->update(sequence_number,out);
}

int BlobDescriptor::Id() const {
	return id_;
}

cv::Rect BlobDescriptor::getLastRawBound() const {
	return *raw_bounds_.rbegin();
}

cv::Rect BlobDescriptor::getRawBound(int index) const {
	try {
		return raw_bounds_.at(index);
	} catch (const std::out_of_range& oor){
		std::cout<<"requested out of range blob"<<std::endl;
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
	return *contours_.rbegin();
}



/*
void BlobDescriptorFetcher::receiver(const dhs::blob& msg) {
	//deserialize the message into the map
	//get colors
	ColorPair colors;
	if(msg.colors.size() >=3) {
		colors.first[0] = msg.colors[0];
		colors.first[1] = msg.colors[1];
		colors.first[2] = msg.colors[2];
	}
	if(msg.colors.size() >=6) {
		colors.second[0] = msg.colors[3];
		colors.second[1] = msg.colors[4];
		colors.second[2] = msg.colors[5];
	}

	//get position/dimensions
	cv::Rect position(msg.filtered_position[0],msg.filtered_position[1],msg.filtered_size[0],msg.filtered_size[1]);
	BlobDescriptorPtr new_blob(new BlobDescriptor(msg.first_seen,msg.id,colors,position,msg.depth,NULL));
	std::pair<int,BlobDescriptorPtr> insert_val(msg.id,new_blob);
	std::pair<std::map<int,BlobDescriptorPtr>::iterator,bool > returned = data_.insert(insert_val);
	//if returned.second then we're done, otherwise need to update the existing blob
	if(!returned.second) {
		returned.first->second->history_.push_back(HistoryDescriptor(msg.last_seen,-1,position));
		returned.first->second->colors_ = colors;
	}

	//deserialize contour
	utility::deSerializeContour(msg.contour,returned.first->second->contour_);
	std::cout<<"Got blob #"<<data_[msg.id]->id_<<std::endl;

	//get centroid
	returned.first->second->centroid_ = cv::Point2f(msg.centroid[0],msg.centroid[1]);

	//add this id to the blobs_updated array so something else can go do processing on it
	blobs_updated_.push_back(msg.id);
}
*/
