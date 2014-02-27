/*
 * ShowBlobs.cpp
    Copyright (C) 2014 Timothy Sweet

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

typedef BlobDescriptorDecoratedKM BlobType;
typedef boost::shared_ptr<BlobType> BlobPtr;
typedef std::vector<BlobPtr> BlobPtrVector;

class Worker {
	//this class is only used to provide data to the ros::timer
public:
	Worker(const std::string& first_topic) :
		input_stream_(first_topic),
		next_id(0),
		handle_() { blobs_in_ = handle_.subscribe("blobs_in",1,&Worker::blobCallback,this);}

	void callback(const ros::TimerEvent& event);

	ImageFetcher input_stream_;

private:
	//holds the id of the next blob to be added to the list
	//can't just use blobs_.size() because some could be culled
	int next_id;
	ros::NodeHandle handle_;
	void blobCallback(dhs::blobPtr msg);
	std::map<int,BlobPtr> blobs_;
	std::vector<int> blobs_updated_;
	ros::Subscriber blobs_in_;

};

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"show_blobs");
	ros::NodeHandle handle("~");
	setLoggerDebug();

	//loop at 60 hz since the camera runs half that fast
	Worker worker("rgb_in");
	ros::Timer timer = handle.createTimer(ros::Duration(1./30.), &Worker::callback, &worker);

	//can be shut down safely with Ctrl+C
	ros::spin();

	return 0;
}

void Worker::callback(const ros::TimerEvent& event) {
	//get the image and draw currently visible blobs on it
	cv::Mat rgb;
	if(input_stream_.GetFrame(rgb)==FRAME_NOT_UPDATED) {
		return;
	}
//	ROS_DEBUG("Grabed frame");

	//draw all the updated blobs
	std::vector<int>::const_iterator blob_it = blobs_updated_.begin();
	while(blob_it !=blobs_updated_.end()) {
		cv::rectangle(rgb,blobs_[*blob_it]->getLastFilteredBound(),cv::Scalar(255),2);
		blob_it++;
	}
	//note: there is a race condition here. It is possible that a blob could be
	//updated between when we last checked and now. This is acceptable for this
	//use case.
	blobs_updated_.clear();

	cv::imshow("Blobs",rgb);
	cv::waitKey(1);
}

void Worker::blobCallback(dhs::blobPtr msg) {
	//find the blob to match this to
	try {
		blobs_.at(msg->id)->deserializeBlob(msg);
	}
	catch (std::out_of_range&) {
		BlobPtr temp(new BlobType(msg->id));
			temp->deserializeBlob(msg);
			blobs_.insert(std::pair<int,BlobPtr>(msg->id,temp));
	}
	blobs_updated_.push_back(msg->id);
}
