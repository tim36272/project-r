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
#include "dhs/Utility.h"
#include "dhs/SynchronousImageFetcher.h"
#include "dhs/BlobDescriptorDecorated.h"
#include "dhs/common.h"

//constants (experimentally chosen)
static const int kMinContourArea = 1500;

typedef BlobDescriptorDecoratedKM BlobType;
typedef boost::shared_ptr<BlobType> BlobPtr;
typedef std::vector<BlobPtr> BlobPtrVector;
typedef BlobPtrVector::iterator BlobPtrVectorIt;
typedef BlobPtrVector::const_iterator BlobPtrVectorConstIt;

class Worker {
	//this class is only used to provide data to the ros::timer
public:
	Worker(const std::string& rgb_segmentation_topic, const std::string& depth_segmentation_topic, const std::string& rgb_topic,const std::string& depth_topic,const std::string& output_topic) :
		input_stream_(rgb_segmentation_topic,depth_segmentation_topic,rgb_topic,depth_topic),
		next_id(0),
		handle_() {
		output_stream_ = handle_.advertise<dhs::blob>(output_topic,100);
	}

	void callback(const ros::TimerEvent& event);

	SynchronousImageFetcher input_stream_;
	ros::Publisher output_stream_;
	BlobPtrVector blobs_;
	ContourList contours_;

	cv::Mat rgb_segmentation_,depth_segmentation_,combined_segmentation_,rgb_,depth_;

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
	setLoggerDebug();

	//loop at 60 hz since the camera runs half that fast
	Worker worker("rgb_segmentation_in","depth_segmentation_in","rgb_in","depth_in","blobs_out");
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
	int sequence_number = input_stream_.GetFrame(rgb_segmentation_,depth_segmentation_,rgb_,depth_);
	if(sequence_number==FRAME_NOT_UPDATED) {
		//there's no new frames waiting
		return;
	}
	cv::bitwise_and(rgb_segmentation_, depth_segmentation_,combined_segmentation_);
	cv::imshow("combined",combined_segmentation_);
	cv::waitKey(1);

	//process the frames
	//first: get significant blobs
	ROS_DEBUG("Finding blobs");
	findBlobs();

	ROS_DEBUG("updating blobs");
	//second: try to update existing blobs
	updateBlobs(sequence_number);

	ROS_DEBUG("adding blobs");
	//third: add remaining blobs as new blobs
	addBlobs(sequence_number);

	ROS_DEBUG("culling blobs");
	//fourth: remove blobs that were only seen for one frame
	cullBlobs(sequence_number);

	ROS_DEBUG("publishing blobs");
	//fifth: publish blobs over ros
	publishBlobs(sequence_number);
}

void Worker::findBlobs() {
	assert(combined_segmentation_.data && rgb_.data);

	//get contours in the frames using cv::findContours
	cv::findContours(combined_segmentation_.clone(),contours_,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	//iterate through the list and remove small contours
	contours_.erase(std::remove_if(contours_.begin(),contours_.end(),isContourSmall),
					contours_.end());
}
void Worker::updateBlobs(int sequence_number) {
	//for each blob in blobs_ find all the contours in contours_ which are in the same place
	BlobPtrVectorIt blob_cursor = blobs_.begin();

	//the number of blobs should be small
	for(;blob_cursor!=blobs_.end();blob_cursor++) {
		ContourList candidates;
		utility::getCandidates((*blob_cursor)->getLastFilteredBound(),contours_,&candidates);

		//now there is a list of contours in candidates which are sufficiently
		//close to the blob or no contours at all (blob disappeared)
		if(candidates.empty()) {
			//there are no candidates, blob disappeared, so stop trying to update
			ROS_DEBUG_STREAM("There is no candidate for "<<(*blob_cursor)->Id());
			continue;
		}

		//if there is more than one candidate try to merge them
		if(candidates.size() > 1) {
			utility::CombineContours(rgb_.size(),&candidates);
		}

		//find the largest candidate
		ContourListIt max_at = utility::findLargestContour(candidates);

		cv::Rect bound = cv::boundingRect(*max_at);

		//get depth position of this contour
		int depth = depth_.at<uchar>(utility::Center(bound));

		//update the blob with the largest contour
		//TODO: check if the blob is the right color
		(*blob_cursor)->update(sequence_number,*max_at,depth);

		candidates.erase(max_at);

		//add candidates back to the contour list
		//utility::merge(contours_,candidates);
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
		int depth = depth_.at<uchar>(utility::Center(bound));

		BlobPtr temp(new BlobType(next_id++));
		temp->update(sequence_number,*contour_cursor,depth);
		assert(temp->Id()==(next_id-1));
		blobs_.push_back(temp);
	}
}

void Worker::cullBlobs(int sequence_number) {
	if(blobs_.empty()) {
		return;
	}
	BlobPtrVectorIt cursor = blobs_.begin();
	BlobPtrVector new_blobs;
	for(;cursor !=blobs_.end();cursor++) {
		int last_seen = (*cursor)->lastSeen();
		int first_seen = (*cursor)->firstSeen();
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
	ROS_DEBUG("publishing blobs");
	BlobPtrVectorConstIt cursor = blobs_.begin();
	for(;cursor!=blobs_.end();cursor++) {
		//don't publish blobs which were not seen this run
		if((*cursor)->lastSeen()!=sequence_number) {
			continue;
		}
		(*cursor)->serializeBlob(output_stream_);
	}
}

