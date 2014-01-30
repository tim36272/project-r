/*
 * BagOnPersonProcessor.cpp
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
#include <sstream>

//library includes
#include <ros/ros.h>

//project includes
#include "Utility.h"
#include "ImageFetcher.h"
#include "ProcessorNode.h"
#include "EventCodes.h"
#include "BlobDescriptor.h"
#include "Periodic.h"

class BlobDescriptorDecorated : public BlobDescriptor{
public:
	typedef BlobDescriptor super;
	BlobDescriptorDecorated(int id) : super::BlobDescriptor(id) {}
	~BlobDescriptorDecorated() {}

	void update_child() {
		//calculate filtered bound
		filter_.update(&*raw_bounds_.rbegin());

		//store filtered bound
		filtered_bounds_.push_back(filter_.bound());

		//calculate moments
		moments_ = cv::moments(getLastContour());
		ROS_DEBUG_STREAM("Calculated moments for id"<<Id()<<", centroid: "<<getCentroid());
	}

	//get an arbitrary filtered bound
	cv::Rect getFilteredBound(int index) const {
		try {
			return filtered_bounds_.at(index);
		} catch (const std::out_of_range& oor){
			std::cout<<"requested out of range blob"<<std::endl;
			assert(false);
		}
		//dummy return to make the compiler happy
		return (cv::Rect());
	}
	//get most recent filtered bound
	cv::Rect getLastFilteredBound() const {
		assert(filtered_bounds_.size()>0);
		return *(filtered_bounds_.rbegin());
	}
	//return the mass center
	cv::Point2f getCentroid() const {
		return cv::Point2f( moments_.m10/moments_.m00 , moments_.m01/moments_.m00 );
	}
	Periodic tracker_;
	Kalman filter_;
	cv::Moments moments_;
private:
	std::vector<cv::Rect> filtered_bounds_;
	//tools
};
typedef boost::shared_ptr<BlobDescriptorDecorated> BlobDescriptorDecPtr;

class BagOnPersonProcessor : public ProcessorNode {
public:
	DISALLOW_DEFAULT_CONSTRUCTION(BagOnPersonProcessor);
	BagOnPersonProcessor(const std::string& blob_topic,const std::string& event_topic,const std::string& rgb_topic);
	void callback(const ros::TimerEvent& event);
	void blobCallback(dhs::contourPtr msg);

	void init();
	cv::VideoWriter writer_;
private:
	DISALLOW_COPY_AND_ASSIGN(BagOnPersonProcessor);
	ros::NodeHandle handle_;
	ros::Subscriber input_stream_;
	std::map<int,BlobDescriptorDecPtr> blobs_;
	std::vector<int> blobs_updated_;
	ImageFetcher rgb_input_stream;
};


int main(int argc, char* argv[]) {
	//connect to ros
	ros::init(argc, argv, "segmentation");
	//set logger level
	setLoggerDebug();
	ros::NodeHandle handle("~");

	BagOnPersonProcessor processor("blobs_in","events_out","rgb_in");
	assert(processor.writer_.isOpened());
	//loop at 60 hz just because that seems like a pretty good rate given
	//that the camera is running at 30 hz
	ros::Timer timer = handle.createTimer(ros::Duration(1./30.), &BagOnPersonProcessor::callback, &processor);

	ros::spin();

	return 0;
}
BagOnPersonProcessor::BagOnPersonProcessor(const std::string& blob_topic,const std::string& event_topic,const std::string& rgb_topic) :
	rgb_input_stream(rgb_topic){
	event_name_ = BagOnPersonName;
	event_code_ = BagOnPersonCode;
	output_topic_ = event_topic;
	init();
	writer_.open("/home/tim/out.avi",CV_FOURCC('D','I','V','X'),15,cv::Size(640,480),true);
	input_stream_ = handle_.subscribe(blob_topic,3,&BagOnPersonProcessor::blobCallback,this);
}

void BagOnPersonProcessor::init() {
	setupOutput();
}

void BagOnPersonProcessor::callback(const ros::TimerEvent& event) {
	//for each blob which was updated this frame
	if(blobs_updated_.size()==0) {
		return;
	}
	//TODO: get the size another way
	cv::Mat output(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
	std::vector<int>::iterator blob_iterator = blobs_updated_.begin();
	for(;blob_iterator!=blobs_updated_.end();) {
		//just get this blob as a pointer
		BlobDescriptorDecPtr current_blob = blobs_[*blob_iterator];
		/*
		 * erase this index from the updated list and reset the iterator. This prevents
		 * jumping past the end of the vector when incrementing the iterator
		 */
		blobs_updated_.erase(blob_iterator);

		//TODO: fix the race condition here in which an empty list which
		//has a blob added to it after here will break the loop
		blob_iterator = blobs_updated_.begin();

		//draw the contour on a mat
		//TODO:get the size another way
		cv::Mat blob_visual(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
		{
			ContourList temp_list;
			temp_list.push_back(current_blob->getLastContour());
			cv::drawContours(blob_visual,temp_list,0,cv::Scalar(255),-1);
		}

		//compute the body axis
		//try computing it as the horizontal projection(as in sum of vertical stacks of pixels)
		utility::floodConcaveRegions(&blob_visual);
		int symmetry_axis;
		{
			std::vector<int> upward_projection;
			utility::getUpwardProjection(blob_visual,&upward_projection);
			int max=0;
			int max_projection_at=0;
			for(int i=0;i<upward_projection.size();i++) {
				if(upward_projection[i] > max) {
					max = upward_projection[i];
					max_projection_at=i;
				}
			}
			//draw max horizontal projection axis
			//cv::line(blob_visual,cv::Point(max_at,0),cv::Point(max_at,480),cv::Scalar(128+64),2);
			symmetry_axis = max_projection_at;
		}


		cv::Mat full_blob(blob_visual.clone());

		//classify each pixel as symmetric or asymmetric
		utility::removeSymmetricRegions(symmetry_axis,&blob_visual);
		utility::recolorNonSymmetricRegions(symmetry_axis,&full_blob); //this is only for the user's visualization

		//show the symmetry line just for the user's edification
		cv::line(full_blob,cv::Point(symmetry_axis,0),cv::Point(symmetry_axis,480),cv::Scalar(200),2);

		//show which regions are symmetric/asymmetric
		cv::imshow("symetric/aysmetric",full_blob);

		//check periodicity and reclassify pixels if necessary
		//algorithm: if there are no similarity measurements for this blob, it must be the first time it's been seen
		if(!current_blob->tracker_.set_up()) {
			ROS_INFO_STREAM("Blob #"<<current_blob->Id()<<" added to periodicity tracker");
			//this is the first time the blob has been seen
			cv::Point2f centroid( current_blob->moments_.m10/current_blob->moments_.m00 , current_blob->moments_.m01/current_blob->moments_.m00 );
			current_blob->tracker_.setup(current_blob->getLastFilteredBound(),centroid,blob_visual);
		} else if(utility::changedMoreThanFactor( //the bound has changed a lot since first view, so reset all the periodicity tracking stuff
				current_blob->getRawBound(0),
				current_blob->getLastRawBound(), 1)) {
			ROS_INFO_STREAM("Blob #"<<current_blob->Id()<<" changed a lot, resetting its track");
			//the blob changed a lot so dump everything and start over
			//this typcially occurs a few times as the blob is entering the scene
			cv::Point2f centroid( current_blob->moments_.m10/current_blob->moments_.m00 , current_blob->moments_.m01/current_blob->moments_.m00 );
			ROS_DEBUG_STREAM("Resetting the tracker for id: "<<current_blob->Id()<<", centroid is: "<<centroid);
			current_blob->tracker_.setup(current_blob->getLastFilteredBound(),centroid,blob_visual);
		} else {
			//blob has already been seen, just check for periodicity
			Point2fVec dst_points;
			dst_points.push_back(current_blob->getCentroid());
			dst_points.push_back(current_blob->getLastRawBound().tl());
			dst_points.push_back(utility::BottomLeft(current_blob->getLastRawBound()));

			//get an affine transformation matrix which maps the current view to the template view
			cv::Mat warp_mat = cv::getAffineTransform(dst_points,current_blob->tracker_.src_points());

			//warp the visual representation of the non symmetric parts of the blob to the reference frame (ideally the first full view of the blob)
			cv::Mat warped_blob;
			cv::warpAffine(blob_visual,warped_blob,warp_mat,blob_visual.size());

			current_blob->tracker_.addFrame(warped_blob);
		}
		ROS_DEBUG("Done checking periodicity");

		//draw interesting regions
		cv::Mat rgb;
		if(current_blob->filter_.initialized() > 0 && rgb_input_stream.GetMostRecentFrame(rgb)!=0) {
			std::cout<<"There is an interesting region"<<std::endl;
			cv::Rect interest_rect(current_blob->getLastRawBound().x+current_blob->filter_.bound().x,
					current_blob->getLastRawBound().y+current_blob->filter_.bound().y,
					current_blob->filter_.bound().width,
					current_blob->filter_.bound().height);
			cv::rectangle(rgb,interest_rect,cv::Scalar(255),1);
			imshow("interesting region",rgb);
			writer_.write(rgb);
		}
		else {
			cv::Mat temp(cv::Size(640,480),CV_8UC3,cv::Scalar::all(0));
			rgb_input_stream.GetMostRecentFrame(temp);
			writer_.write(temp);
		}

	}
	cv::waitKey(1);
}

void BagOnPersonProcessor::blobCallback(dhs::contourPtr msg) {
	//find the blob to match this to
	try {
		blobs_.at(msg->id)->deserializeContour(msg->header.seq,msg->contour);
	}
	catch (std::out_of_range&) {
		BlobDescriptorDecPtr temp(new BlobDescriptorDecorated(msg->id));
			temp->deserializeContour((int)msg->header.seq,msg->contour);
			blobs_.insert(std::pair<int,BlobDescriptorDecPtr>(msg->id,temp));
	}
	blobs_updated_.push_back(msg->id);
}
