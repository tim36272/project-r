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

HistoryDescriptor::HistoryDescriptor(int timestamp,int depth_position, const cv::Rect& location):
	timestamp_(timestamp),
	depth_position_(depth_position),
	location_(location) {}

BlobDescriptor::BlobDescriptor(int sequence_number, int id, const ColorPair& colors,const cv::Rect& location,int depth,Contour* contour):
	filter_(location),
	history_(1,HistoryDescriptor(sequence_number,depth,location)),
	id_(id),
	colors_(colors){
	//TODO: modify BlobDescriptorFetcher so NULL isn't necessary
	if(contour!=NULL) {
		contour_.swap(*contour);
		moments_ = cv::moments(contour_,false);
		centroid_ = utility::centroid(moments_);
	}
}

//Sets upper/lower halves (maybe)
//updates last_seen
//updates history (maybe)
void BlobDescriptor::update(int sequence_number, const cv::Rect& location,int depth,Contour* contour) {
	filter_.update(&location);
	history_.push_back(HistoryDescriptor(sequence_number,depth,location));
	//TODO: modify BlobDescriptorFetcher so NULL check isn't necessary
	if(contour!=NULL) {
		contour_.swap(*contour);
		moments_ = cv::moments(contour_,false);
		centroid_ = utility::centroid(moments_);
	}
}

cv::Rect BlobDescriptor::LastRawBound() const {
	return history_.rbegin()->location_;
}
cv::Rect BlobDescriptor::CurrentBound() const {
	return filter_.bound();
}
//timestamp of end(history)
int BlobDescriptor::LastSeen() const {
	return history_.rbegin()->timestamp_;
}
//timestamp of begin(history)
int BlobDescriptor::FirstSeen() const {
	return history_.begin()->timestamp_;
}
ColorPair BlobDescriptor::Colors() const{
	return colors_;
}

int BlobDescriptor::Id() const {
	return id_;
}

int BlobDescriptor::getDepth() const {
	return history_.rbegin()->depth_position_;
}

cv::Point2f BlobDescriptor::getCentroid() const {
	return centroid_;
}

cv::Rect BlobDescriptor::getBound(int index) const {
	try {
		return history_.at(index).location_;
	} catch (const std::out_of_range& oor){
		std::cout<<"requested out of range blob"<<std::endl;
		assert(false);
	}
	return (cv::Rect());
}

BlobDescriptorFetcher::BlobDescriptorFetcher(const std::string& topic) {
	blob_subscriber_ = handle_.subscribe(topic,100,&BlobDescriptorFetcher::receiver,this);
}
std::string BlobDescriptorFetcher::getTopic() {
	return blob_subscriber_.getTopic();
}

void BlobDescriptorFetcher::receiver(const dhs::blob& msg) {
	std::cout<<"getting a blob"<<std::endl;
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
	std::cout<<"inserted"<<std::endl;
	if(!returned.second) {
		std::cout<<"updating"<<std::endl;
		returned.first->second->history_.push_back(HistoryDescriptor(msg.last_seen,-1,position));
		returned.first->second->colors_ = colors;
	}

	//deserialize contour
	utility::deSerializeContour(msg.contour,returned.first->second->contour_);
	std::cout<<"Got blob with ID: "<<data_[msg.id]->id_<<std::endl;

	//get centroid
	returned.first->second->centroid_ = cv::Point2f(msg.centroid[0],msg.centroid[0]);

	//add this id to the blobs_updated array so something else can go do processing on it
	blobs_updated_.push_back(msg.id);
}
