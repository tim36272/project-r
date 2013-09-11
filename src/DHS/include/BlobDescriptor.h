#ifndef TIM_SRC_BLOBDESCRIPTOR_H
#define TIM_SRC_BLOBDESCRIPTOR_H

#include <list>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Kalman.h"
#include "Types.h"

class HistoryDescriptor {
public:
	cv::Rect position;

	friend std::ostream& operator<<(std::ostream& out,HistoryDescriptor &object) {
		out<<object.position;
		return out;
	}
};

class PersonDescriptor{
public:
	cv::Rect last_location;
	int last_seen, first_seen, observation_count,depth_position;
	std::map<int,HistoryDescriptor> history;
	Kalman filter;
	PersonDescriptor();
	PersonDescriptor(const PersonDescriptor& rhs);
	PersonDescriptor& operator=(const PersonDescriptor& rhs);
	std::ostream& operator << (std::ostream& out);
	void update(const PersonDescriptor& rhs);
	void init_kalman();
	void update_kalman(int frame_number);
	utility::Pair_<cv::Scalar> color;
	cv::Rect last_upper_location,last_lower_location;

};

class BagDescriptor{
public:
	cv::Rect last_location;
	int last_seen, first_seen, observation_count, depth_position;
	std::map<int,HistoryDescriptor> history;
	Kalman filter;
	BagDescriptor();
	BagDescriptor(const BagDescriptor& rhs);
	BagDescriptor& operator=(const BagDescriptor& rhs);
	std::ostream& operator << (std::ostream& out);
	void update(const BagDescriptor& rhs);
	void init_kalman();
	void update_kalman(int frame_number);
	cv::Scalar color;
	cv::Rect last_upper_location,last_lower_location;
	int belongs_to;
};
#endif
