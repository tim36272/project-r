/*
 * BagInteraction.cpp
    Copyright (C) 2014  Timothy Sweet

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


//project includes
#include "dhs/Utility.h"
#include "dhs/common.h"
#include "dhs/BlobDescriptorDecorated.h"
#include "dhs/ImageFetcher.h"
#include "dhs/blob.h"
#include "dhs/Interaction.h"

typedef BlobDescriptorDecoratedKBMT BlobType;
typedef boost::shared_ptr<BlobType> BlobPtr;
typedef std::map<int,BlobPtr> BlobPtrMap;
typedef BlobPtrMap::iterator BlobPtrMapIt;

class Worker {
	//this class is only used to provide data to the ros::timer
public:
	Worker(const std::string& blob_topic) :
		next_id(0),
		handle_(),
		rgb_in_stream_("/camera/rgb/image_color"){ blobs_in_ = handle_.subscribe(blob_topic,1,&Worker::blobCallback,this);}

	void callback(const ros::TimerEvent& event);

private:
	//holds the id of the next blob to be added to the list
	//can't just use blobs_.size() because some could be culled
	int next_id;
	ros::NodeHandle handle_;
	void blobCallback(dhs::blobPtr msg);
	BlobPtrMap blobs_;
	std::vector<int> blobs_updated_;
	ros::Subscriber blobs_in_;
	interaction::Interactions interactions_;
	//temporarily grab the rgb stream here, eventually the output will be directed to UI
	ImageFetcher rgb_in_stream_;

};

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"bag_interaction");
	ros::NodeHandle handle("~");
	setLoggerDebug();

	//loop at 60 hz since the camera runs half that fast
	Worker worker("blobs_in");
	ros::Timer timer = handle.createTimer(ros::Duration(1./10.), &Worker::callback, &worker);

	//can be shut down safely with Ctrl+C
	ros::spin();

	return 0;
}

void Worker::callback(const ros::TimerEvent& event) {
	//note: blobs_updated_ needs to be consumed by something, in this case the periodicity tracker
	if(blobs_.empty() || blobs_updated_.empty()) {
		return;
	}
//	ROS_DEBUG_STREAM("There are "<<blobs_.size()<<" blobs");
	//for every blob, determine its relationships
	BlobPtrMapIt blob_it = blobs_.begin();
	while(blob_it !=blobs_.end()) {
		//make a list of blobs for this one to interact with
		std::vector<BlobPtr> other_blobs = utility::mapToVectorExcludeOne(blobs_,blob_it);

		interaction::checkForInteractions(blob_it->second, other_blobs,interactions_);
		++blob_it;
	}
	interaction::print(interactions_);

	//backpack module
	cv::Mat rgb(cv::Size(640,480),CV_8UC3,cv::Scalar::all(0));
	rgb_in_stream_.GetMostRecentFrame(rgb);
	periodic::update(rgb,blobs_,blobs_updated_);
}

void Worker::blobCallback(dhs::blobPtr msg) {
	//find the blob to match this to
	try {
		//the blob exists
		blobs_.at(msg->id)->deserializeBlob(msg);
	}
	catch (std::out_of_range&) {
		//the blob does not exist yet
		BlobPtr temp(new BlobType(msg->id));
			temp->deserializeBlob(msg);
			if(utility::isBagSized(temp)) {
				utility::assignBagOwner(blobs_,temp);
				temp->set_bag();
			}
			blobs_.insert(std::pair<int,BlobPtr>(msg->id,temp));
	}
	blobs_updated_.push_back(msg->id);
}
