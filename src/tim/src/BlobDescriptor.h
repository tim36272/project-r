#ifndef TIM_SRC_BLOBDESCRIPTOR_H
#define TIM_SRC_BLOBDESCRIPTOR_H

#include <list>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Kalman.h"

#define NO_OWNER -1

struct PositionDescriptor {
	cv::Rect position;
	int timestamp;
};

struct BlobDescriptor {
	cv::MatND upper,lower;
	int last_seen;
	int first_seen;
	cv::Rect last_location;
	std::list<PositionDescriptor> history;
	cv::Mat image;
	cv::Mat upper_hue_plane,lower_hue_plane;
	int belongs_to;
	Kalman filter;
	BlobDescriptor() {
		last_seen=first_seen=-1;
		belongs_to = NO_OWNER;
		last_location = cv::Rect();
	}
	BlobDescriptor(const BlobDescriptor& rhs) {
		image = rhs.image;
		last_seen = rhs.last_seen;
		first_seen = rhs.first_seen;
		last_location = rhs.last_location;
		rhs.upper.copyTo(upper);
		rhs.lower.copyTo(lower);
		belongs_to = rhs.belongs_to;
		//copy the list
		history = rhs.history;
		filter = rhs.filter;
		upper_hue_plane = rhs.upper_hue_plane;
		lower_hue_plane = rhs.lower_hue_plane;

	}
	BlobDescriptor& operator=(const BlobDescriptor& rhs) {
		image = rhs.image;
		last_seen = rhs.last_seen;
		first_seen = rhs.first_seen;
		last_location = rhs.last_location;
		rhs.upper.copyTo(upper);
		rhs.lower.copyTo(lower);
		belongs_to = rhs.belongs_to;
		//copy the list
		history = rhs.history;
		filter = rhs.filter;
		upper_hue_plane = rhs.upper_hue_plane;
				lower_hue_plane = rhs.lower_hue_plane;
		return *this;
	}
	std::ostream& operator << (std::ostream& out) {
		out <<"First seen: "<<first_seen<<"Last seen: "<<last_seen<<" at: "<<last_location<<" belongs to: "<<belongs_to;
		return out;
	}
	void update(const BlobDescriptor& rhs) {
		image = rhs.image;
		last_seen = rhs.last_seen;
		//don't update first_seen
		last_location = rhs.last_location;
		upper = rhs.upper;
		lower = rhs.lower;
//		upper = upper*0.8+rhs.upper*0.2;
//		lower = lower*0.8+rhs.lower*0.2;
///		belongs_to = rhs.belongs_to;
		//add the location to the internal list
		PositionDescriptor temp;
		temp.position = last_location;
		temp.timestamp = last_seen;
		history.push_back(temp);
		upper_hue_plane = rhs.upper_hue_plane;
						lower_hue_plane = rhs.lower_hue_plane;
	}
	void init_kalman() {
		filter.init(last_location);
	}
	void update_kalman(int frame_number) {
		if(last_seen==frame_number) {
			filter.update(&last_location);
		}
		else {
			//if this is a peson or recently moving bag
			if((belongs_to == NO_OWNER) || ((frame_number-last_seen) < 4)) {
				filter.update(NULL);
			}

		}
	}
};
#endif
