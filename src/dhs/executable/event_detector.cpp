/*
 * event_detector.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: tim
 */

#include <ros/ros.h>
#include "dhs/blob.h"
#include "dhs/BlobDescriptorDecorated.h"

typedef BlobDescriptorDecoratedKB BlobType;
typedef boost::shared_ptr<BlobType> BlobPtr;
typedef std::map<int,BlobPtr> BlobPtrMap;
typedef BlobPtrMap::iterator BlobPtrMapIt;

class Worker {
	//this class is only used to provide data to the ros::timer
public:
	Worker(const std::string& blob_topic) :
		next_id(0),
		handle_() { blobs_in_ = handle_.subscribe(blob_topic,1,&Worker::blobCallback,this);}

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

};

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"event_detector");
	ros::NodeHandle handle("~");
	setLoggerDebug();

	//loop at 30 hz
	Worker worker("blobs_in");
	ros::Timer timer = handle.createTimer(ros::Duration(1./30.), &Worker::callback, &worker);

	//can be shut down safely with Ctrl+C
	ros::spin();

	return 0;
}

void Worker::callback(const ros::TimerEvent& event) {
	//TODO: need to flag contours as bags or people, and interpret ownership
	//look for events between all currently visible blobs
	BlobPtrMapIt first_it=blobs_.begin(),second_it=blobs_.begin();
	while(first_it!=blobs_.end()) {
		while(second_it!=blobs_.end()) {
			if(first_it==second_it) {
				++second_it;
				continue;
			}
			//compute things about these blobs

			++second_it;
		}
		++first_it;
	}
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
