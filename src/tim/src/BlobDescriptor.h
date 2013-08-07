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

class BlobDescriptor {
public:
	cv::MatND upper,lower;
	int last_seen;
	int first_seen;
	cv::Rect last_location;
	std::list<PositionDescriptor> history;
	cv::Mat image;
	cv::Mat upper_hue_plane,lower_hue_plane;
	int belongs_to;
	Kalman filter;
	cv::Moments moments_;
	BlobDescriptor();
	BlobDescriptor(const BlobDescriptor& rhs);
	BlobDescriptor& operator=(const BlobDescriptor& rhs);
	std::ostream& operator << (std::ostream& out);
	void update(const BlobDescriptor& rhs);
	void init_kalman();
	void update_kalman(int frame_number);
};
#endif
