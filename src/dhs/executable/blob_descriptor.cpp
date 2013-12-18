/*
 * blob_descriptor.cpp
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

//std includes
#include <algorithm>
#include <boost/shared_ptr.hpp>

//library includes
#include <ros/ros.h>
#include "dhs/blob.h"

//project includes
#include "../include/Utility.h"
#include "../include/SynchronousImageFetcher.h"
#include "../include/BlobDescriptor.h"

//constants
//experimentally chosen
static const int kMinContourArea = 1000;

class Worker {
	//this class is only used to provide data to the ros::timer
public:
	Worker(const std::string& first_topic, const std::string& second_topic,const std::string& third_topic,const std::string& output_topic) :
		input_stream_(first_topic,second_topic,third_topic),
		next_id(0),
		handle_() {
		output_stream_ = handle_.advertise<dhs::blob>(output_topic,100);
	}

	void callback(const ros::TimerEvent& event);

	SynchronousImageFetcher input_stream_;
	ros::Publisher output_stream_;
	BlobDescriptorPtrVector blobs_;
	ContourList contours_;

	cv::Mat segmentation_,rgb_,depth_;

private:
	void findBlobs();
	void updateBlobs(int sequence_number);
	void addBlobs(int sequence_number);
	void cullBlobs(int sequence_number);
	void publishBlobs(int sequence_number);
	//holds the id of the next blob to be added to the list
	//can't just use blobs_.size() because some could be culled
	int next_id;
	ros::NodeHandle handle_;

};

bool isContourSmall(const Contour& rhs) {
	return cv::contourArea(rhs) < kMinContourArea;
}

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"blob_descriptor");
	ros::NodeHandle handle("~");

	//loop at 60 hz since the camera runs half that fast
	Worker worker("segmentation_in","rgb_in","depth_in","blobs_out");
	ros::Timer timer = handle.createTimer(ros::Duration(1./120.), &Worker::callback, &worker);

	//can be shut down safely with Ctrl+C
	ros::spin();

	return 0;
}

void Worker::callback(const ros::TimerEvent& event) {
	//this callback ideally runs at 30hz
	//we can check TimerEvent if current_expected-current_real is less than 30hz, which means we are more than a frame behind
	//TODO: check if this is running realtime

	//wait until we get a frame
	int sequence_number = input_stream_.GetFrame(segmentation_,rgb_,depth_);
	if(sequence_number==FRAME_NOT_UPDATED) {
		//there's no new frames waiting
		return;
	}

	//process the frames
	//first: get significant blobs
//	std::cout<<"finding blobs"<<std::endl;
	findBlobs();

//	std::cout<<"updating blobs"<<std::endl;
	//second: try to update existing blobs
	updateBlobs(sequence_number);

//	std::cout<<"adding blobs"<<std::endl;
	//third: add remaining blobs as new blobs
	addBlobs(sequence_number);

//	std::cout<<"culling blobs"<<std::endl;
	//fourth: remove blobs that were only seen for one frame
	cullBlobs(sequence_number);

//	std::cout<<"publishing blobs"<<std::endl;
	//fifth: publish blobs over ros
	publishBlobs(sequence_number);

	//testing: show the blobs
	cv::Mat test_output(rgb_.size(),CV_8UC3,cv::Scalar(0));
	for(int i=0;i<blobs_.size();i++) {
		//draw the blob's bounding rect
		cv::rectangle(test_output,blobs_[i]->CurrentBound(),cv::Scalar(rand()%128+128,rand()%128+128,rand()%128+128),5);
	}
}

void Worker::findBlobs() {
	assert(segmentation_.data && rgb_.data);

	//get contours in the frames using cv::findContours
	cv::findContours(segmentation_.clone(),contours_,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	//iterate through the list and remove small contours
	contours_.erase(std::remove_if(contours_.begin(),contours_.end(),isContourSmall),
					contours_.end());
}
void Worker::updateBlobs(int sequence_number) {
	//for each blob in blobs_ find all the contours in contours_ which are in the same place
	BlobDescriptorPtrVectorIt blob_cursor = blobs_.begin();

	//the number of blobs should be small
	for(;blob_cursor!=blobs_.end();blob_cursor++) {
//		std::cout<<"getting candidates"<<std::endl;
		ContourList candidates;
		utility::getCandidates((*blob_cursor)->CurrentBound(),contours_,&candidates);

		//now there is a list of contours in candidates which are sufficiently
		//close to the blob or no contours at all (blob disappeared)
		if(candidates.empty()) {
			//there are no candidates, blob disappeared, so stop trying to update
			continue;
		}

		//if there is more than one candidate try to merge them
		if(candidates.size() > 1) {
			//This next function is not yet implemented
			utility::CombineContours(rgb_.size(),&candidates);
		}

		//find the largest candidate
		ContourListIt max_at = utility::findLargestContour(candidates);

		//get bounding box of contour
		cv::Rect bound = cv::boundingRect(*max_at);

		//get depth position of this contour
		int depth = depth_.at<uchar>(utility::Center(bound));

		//update the blob with the largest contour
		//TODO: check if the blob is the right color
		(*blob_cursor)->update(sequence_number,bound,depth,&*max_at);

		candidates.erase(max_at);

		//add candidates back to the contour list
		utility::merge(contours_,candidates);
	}


}
void Worker::addBlobs(int sequence_number) {
	//for every contour, create a new blob in blobs_
	ContourListIt contour_cursor = contours_.begin();
	for(;contour_cursor!=contours_.end();contour_cursor++) {
		//seq #, ID, colors, location
		cv::Rect bound = cv::boundingRect(*contour_cursor);

		//TODO: look for average color instead of point color
		ColorPair colors;
		colors.first = utility::At(rgb_,utility::Center(utility::UpperHalf(bound)));
		colors.second = utility::At(rgb_,utility::Center(utility::LowerHalf(bound)));

		//get depth position of this contour
		int depth = 0;//depth_.at<uchar>(utility::Center(bound));

		BlobDescriptorPtr temp(new BlobDescriptor(sequence_number,next_id++,colors,bound,depth,&*contour_cursor));
		assert(temp->Id()==(next_id-1));
		blobs_.push_back(temp);
	}
}

void Worker::cullBlobs(int sequence_number) {
	if(blobs_.empty()) {
		return;
	}
	BlobDescriptorPtrVectorIt cursor = blobs_.begin();
	BlobDescriptorPtrVector new_blobs;
	for(;cursor !=blobs_.end();cursor++) {
		int last_seen = (*cursor)->LastSeen();
		int first_seen = (*cursor)->FirstSeen();
		if(first_seen!=sequence_number) {
			if(last_seen-first_seen!= 0) {
				//it is old enough, add it to the new blobs list
				new_blobs.push_back(*cursor);
			}
		}
		else {
			//it is brand new so keep it
			new_blobs.push_back(*cursor);
		}
	}
	blobs_.swap(new_blobs);
}

void Worker::publishBlobs(int sequence_number) {
	BlobDescriptorPtrVectorConstIt cursor = blobs_.begin();
	for(;cursor!=blobs_.end();cursor++) {
		if((*cursor)->LastSeen()!=sequence_number) {
			continue;
		}
		//generate the message
		dhs::blobPtr msg(new dhs::blob());
		msg->colors.push_back((*cursor)->Colors().first[0]);
		msg->colors.push_back((*cursor)->Colors().first[1]);
		msg->colors.push_back((*cursor)->Colors().first[2]);
		msg->colors.push_back((*cursor)->Colors().second[0]);
		msg->colors.push_back((*cursor)->Colors().second[1]);
		msg->colors.push_back((*cursor)->Colors().second[2]);

		msg->filtered_position[0] = (*cursor)->CurrentBound().x;
		msg->filtered_position[1] = (*cursor)->CurrentBound().y;

		msg->filtered_size[0] = (*cursor)->CurrentBound().width;
		msg->filtered_size[1] = (*cursor)->CurrentBound().height;

		msg->first_seen = (*cursor)->FirstSeen();
		msg->id = (*cursor)->Id();
		msg->last_seen = (*cursor)->LastSeen();

		msg->depth = (*cursor)->getDepth();

		msg->centroid[0] = (*cursor)->getCentroid().x;
		msg->centroid[1] = (*cursor)->getCentroid().y;


		utility::serializeContour((*cursor)->contour_,msg->contour);

		//TODO:put in contour points
		output_stream_.publish(msg);

	}
}

