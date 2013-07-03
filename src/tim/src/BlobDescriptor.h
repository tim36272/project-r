#ifndef TIM_SRC_BLOBDESCRIPTOR_H
#define TIM_SRC_BLOBDESCRIPTOR_H

#include <list>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

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
	int belongs_to;
	cv::KalmanFilter position_kalman,dimension_kalman;
	cv::Mat location_estimate,dimension_estimate;
	BlobDescriptor() {
		last_seen=first_seen=-1;
		belongs_to = NO_OWNER;
		last_location = cv::Rect();

		//setup kalman
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
		position_kalman = rhs.position_kalman;
		dimension_kalman = rhs.dimension_kalman;
	}
	BlobDescriptor& operator=(const BlobDescriptor& rhs) {
		//try not to use this
		assert(false);
		image = rhs.image;
		last_seen = rhs.last_seen;
		first_seen = rhs.first_seen;
		last_location = rhs.last_location;
		upper = rhs.upper;
		lower = rhs.lower;
		belongs_to = rhs. belongs_to;
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
//		upper = upper*0.5+rhs.upper*0.5;
//		lower = lower*0.5+rhs.lower*0.5;
		belongs_to = rhs.belongs_to;
		//add the location to the internal list
		PositionDescriptor temp;
		temp.position = last_location;
		temp.timestamp = last_seen;
		history.push_back(temp);
	}
	void init_kalman() {
		//position
		position_kalman.init(4,2,0);
		cv::setIdentity(position_kalman.measurementMatrix, cv::Scalar::all(1));
		setIdentity(position_kalman.processNoiseCov, cv::Scalar::all(0.00001));
		setIdentity(position_kalman.measurementNoiseCov, cv::Scalar::all(0.1));
		setIdentity(position_kalman.errorCovPost, cv::Scalar::all(.1));

		cv::Mat transition(4,4,CV_32FC1);
		cv::setIdentity(transition,cv::Scalar(1));
		transition.at<float>(0,2) = 1;
		transition.at<float>(1,3) = 1;
		position_kalman.transitionMatrix = transition;

		//initialize the Kalman filter
		float x = last_location.x+last_location.width/2;
		float y = last_location.y+last_location.height/2;
		position_kalman.statePre.at<float>(0) = x;
		position_kalman.statePre.at<float>(1) = y;
		position_kalman.statePre.at<float>(2) = 0;
		position_kalman.statePre.at<float>(3) = 0;

		//dimensions
		dimension_kalman.init(4,2,0);
		cv::setIdentity(dimension_kalman.measurementMatrix, cv::Scalar::all(1));
		setIdentity(dimension_kalman.processNoiseCov, cv::Scalar::all(0.00001));
		setIdentity(dimension_kalman.measurementNoiseCov, cv::Scalar::all(0.1));
		setIdentity(dimension_kalman.errorCovPost, cv::Scalar::all(.1));

		dimension_kalman.transitionMatrix = transition;

		//initialize the Kalman filter
		float width = last_location.width;
		float height = last_location.height;
		dimension_kalman.statePre.at<float>(0) = width;
		dimension_kalman.statePre.at<float>(1) = height;
		dimension_kalman.statePre.at<float>(2) = 0;
		dimension_kalman.statePre.at<float>(3) = 0;
	}
	void update_kalman() {
		//position
		float x = last_location.x+last_location.width/2;
		float y = last_location.y+last_location.height/2;

		cv::Mat measurement(2,1,CV_32FC1);
		measurement.at<float>(0)=x;
		measurement.at<float>(1)=y;
		location_estimate = position_kalman.predict();
		position_kalman.correct(measurement);


		//dimensions
		float width = last_location.width;
		float height = last_location.height;

		measurement.at<float>(0)=width;
		measurement.at<float>(1)=height;
		dimension_estimate = dimension_kalman.predict();
		dimension_kalman.correct(measurement);
	}
};
#endif
