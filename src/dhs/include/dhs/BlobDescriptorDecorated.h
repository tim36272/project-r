/*
 * BlobDescriptorDecorated.h
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

#include "dhs/BlobDescriptor.h"
#include "dhs/Periodic.h"
/*
 * Decorated BlobDescriptor includes Kalman Filter and Moments
 */
class BlobDescriptorDecoratedKM : public BlobDescriptor{
public:
	typedef BlobDescriptor super;
	BlobDescriptorDecoratedKM(int id) : super::BlobDescriptor(id) {}
	~BlobDescriptorDecoratedKM() {}

	void update(int sequence_number, Contour& swapped_contour) {
		super::update(sequence_number,swapped_contour);

		//calculate filtered bound
		filter_.update(&*raw_bounds_.rbegin());

		//store filtered bound
		filtered_bounds_.push_back(filter_.bound());
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
		return *filtered_bounds_.rbegin();
	}
	//return the mass center
	cv::Point2f getCentroid() const {
		return cv::Point2f( moments_.m10/moments_.m00 , moments_.m01/moments_.m00 );
	}
private:
	std::vector<cv::Rect> filtered_bounds_;
	//tools
	Kalman filter_;
	cv::Moments moments_;

	void serialize_decorators(dhs::blobPtr msg) {
		msg->moments[0] = moments_.m00;
		msg->moments[1] = moments_.m01;
		msg->moments[2] = moments_.m02;
		msg->moments[3] = moments_.m03;
		msg->moments[4] = moments_.m10;
		msg->moments[5] = moments_.m11;
		msg->moments[6] = moments_.m12;
		msg->moments[7] = moments_.m20;
		msg->moments[8] = moments_.m21;
		msg->moments[9] = moments_.m30;

		msg->filtered_position[0] = getLastFilteredBound().x;
		msg->filtered_position[1] = getLastFilteredBound().y;
		msg->filtered_size[0] = getLastFilteredBound().width;
		msg->filtered_size[1] = getLastFilteredBound().height;
	}
	void deserialize_decorators(dhs::blobPtr msg) {
		moments_.m00 = msg->moments[0];
		moments_.m01 = msg->moments[1];
		moments_.m02 = msg->moments[2];
		moments_.m03 = msg->moments[3];
		moments_.m10 = msg->moments[4];
		moments_.m11 = msg->moments[5];
		moments_.m12 = msg->moments[6];
		moments_.m20 = msg->moments[7];
		moments_.m21 = msg->moments[8];
		moments_.m30 = msg->moments[9];

		filtered_bounds_.push_back(cv::Rect(msg->filtered_position[0],
											msg->filtered_position[1],
											msg->filtered_size[0],
											msg->filtered_size[1]));
	}
};

/*
 * Decorated BlobDescriptor includes Kalman Filter and Moments
 */
class BlobDescriptorDecoratedKMT : public BlobDescriptor{
public:
	typedef BlobDescriptor super;
	BlobDescriptorDecoratedKMT(int id) : super::BlobDescriptor(id) {}
	~BlobDescriptorDecoratedKMT() {}

	void update(int sequence_number, Contour& swapped_contour) {
		super::update(sequence_number,swapped_contour);

		//calculate filtered bound
		filter_.update(&*raw_bounds_.rbegin());

		//store filtered bound
		filtered_bounds_.push_back(filter_.bound());
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
		return *filtered_bounds_.rbegin();
	}
	//return the mass center
	cv::Point2f getCentroid() const {
		return cv::Point2f( moments_.m10/moments_.m00 , moments_.m01/moments_.m00 );
	}
	Kalman filter_;
	cv::Moments moments_;
	Periodic tracker_;
private:
	std::vector<cv::Rect> filtered_bounds_;
	//tools




	void serialize_decorators(dhs::blobPtr msg) {
		msg->moments[0] = moments_.m00;
		msg->moments[1] = moments_.m01;
		msg->moments[2] = moments_.m02;
		msg->moments[3] = moments_.m03;
		msg->moments[4] = moments_.m10;
		msg->moments[5] = moments_.m11;
		msg->moments[6] = moments_.m12;
		msg->moments[7] = moments_.m20;
		msg->moments[8] = moments_.m21;
		msg->moments[9] = moments_.m30;

		msg->filtered_position[0] = getLastFilteredBound().x;
		msg->filtered_position[1] = getLastFilteredBound().y;
		msg->filtered_size[0] = getLastFilteredBound().width;
		msg->filtered_size[1] = getLastFilteredBound().height;
	}
	void deserialize_decorators(dhs::blobPtr msg) {
		moments_.m00 = msg->moments[0];
		moments_.m01 = msg->moments[1];
		moments_.m02 = msg->moments[2];
		moments_.m03 = msg->moments[3];
		moments_.m10 = msg->moments[4];
		moments_.m11 = msg->moments[5];
		moments_.m12 = msg->moments[6];
		moments_.m20 = msg->moments[7];
		moments_.m21 = msg->moments[8];
		moments_.m30 = msg->moments[9];

		filtered_bounds_.push_back(cv::Rect(msg->filtered_position[0],
											msg->filtered_position[1],
											msg->filtered_size[0],
											msg->filtered_size[1]));
	}
};
