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
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "Kalman.h"
#include "common.h"
#include "dhs/blob.h"

//a small helper class
class HistoryDescriptor {
public:
	//disallow default constructor
	HistoryDescriptor();
	HistoryDescriptor(int timestamp,int depth_position, const cv::Rect& location);
	//holds the sequence number of this observation
	int timestamp_;
	//holds the position at timestamp_ time
	cv::Rect location_;
	//holds depth position, if applicable
	int depth_position_;
};

class BlobDescriptorFetcher;

class BlobDescriptor {
	public:
	DISALLOW_COPY_AND_ASSIGN(BlobDescriptor);
	BlobDescriptor() {}
	BlobDescriptor(int sequence_number, int id, const ColorPair& colors,const cv::Rect& location,int depth,Contour* contour);
	//Sets upper/lower halves (maybe)
	//updates last_seen
	//updates history (maybe)
	void update(int sequence_number,const cv::Rect& location, int depth,Contour* contour);

	//accessors
	//history_.rbegin->location_
	cv::Rect LastRawBound() const;
	//filter.bounding_rect()
	cv::Rect CurrentBound() const;
	//history_.rbegin->timestamp_
	int LastSeen() const;
	//history_.begin->timestamp_
	int FirstSeen() const;
	//colors_
	ColorPair Colors() const;
	//history_.begin->timestamp_
	int LastSequenceNumber() const;
	//id_
	int Id() const;
	//history.rbegin->depth_position_;
	int getDepth() const;
	//centroid_
	cv::Point2f getCentroid() const;
	//get an arbitrary bound
	cv::Rect getBound(int index) const;

	Contour contour_;
	private:
	int id_;
	std::vector<HistoryDescriptor> history_;
	Kalman filter_;
	ColorPair colors_;
	cv::Moments moments_;
	cv::Point2f centroid_;
	friend class BlobDescriptorFetcher;

};

class BlobDescriptorFetcher {
public:
	DISALLOW_COPY_AND_ASSIGN(BlobDescriptorFetcher);
	BlobDescriptorFetcher(const std::string& topic);
	std::map<int,BlobDescriptorPtr > data_;
	void receiver(const dhs::blob& msg);
	std::string getTopic();
	std::vector<int> blobs_updated_;
private:
	ros::NodeHandle handle_;
	ros::Subscriber blob_subscriber_;
};

#endif /* BLOBDESCRIPTOR_H_ */
