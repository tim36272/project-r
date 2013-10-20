#include "Kalman.h"
void Kalman::init(const cv::Rect& measurement) {
	//initialize the filters
	position_filter_.init(4,2,0);
	dimension_filter_.init(4,2,0);

	//setup the transition matrix
	cv::Mat transition(4,4,CV_32FC1);
	cv::setIdentity(transition,cv::Scalar(1));
	transition.copyTo(dimension_filter_.transitionMatrix);
	transition.at<float>(0,2) = 1;
	transition.at<float>(1,3) = 1;
		//apply it to both filters because they are 4 dimensional, 2 measurement filters
	transition.copyTo(position_filter_.transitionMatrix);


	//initialize filter's internals
		//position
//		std::cout<<"Measurement matrix before: "<<position_filter_.measurementMatrix<<std::endl;
		cv::setIdentity(position_filter_.measurementMatrix, cv::Scalar(1)); //1 //2 rows x 4 cols
//		std::cout<<"Measurement matrix after: "<<position_filter_.measurementMatrix<<std::endl;
		cv::setIdentity(position_filter_.processNoiseCov, cv::Scalar(0.001)); //.00001 //
//		std::cout<<"process noise cov: "<<position_filter_.processNoiseCov<<std::endl;
		cv::setIdentity(position_filter_.measurementNoiseCov, cv::Scalar(0.9)); //.1 a low number here favors measurements over history //.00005
//		std::cout<<"meas noise cov: "<<position_filter_.measurementNoiseCov<<std::endl;
		cv::setIdentity(position_filter_.errorCovPost, cv::Scalar(.1)); //.1
//		std::cout<<"error cov post: "<<position_filter_.errorCovPost<<std::endl;
//		cv::setIdentity(position_filter_.errorCovPre, cv::Scalar::all(.5)); //none
//		cv::setIdentity(position_filter_.gain, cv::Scalar::all(.9)); //none



		//dimensions
		cv::setIdentity(dimension_filter_.measurementMatrix, cv::Scalar(1)); //1
		cv::setIdentity(dimension_filter_.processNoiseCov, cv::Scalar(0.00001)); //.00001
		cv::setIdentity(dimension_filter_.measurementNoiseCov, cv::Scalar(0.001)); //.1
		cv::setIdentity(dimension_filter_.errorCovPost, cv::Scalar(.1)); //.1
//		cv::setIdentity(dimension_filter_.errorCovPre, cv::Scalar::all(.5));//none
//		cv::setIdentity(dimension_filter_.gain, cv::Scalar::all(.9)); //none

	//initialize the state matrices
		//position
		position_filter_.statePost.at<float>(0) = measurement.x+measurement.width/2;
		position_filter_.statePost.at<float>(1) = measurement.y+measurement.height/2;
		position_filter_.statePost.at<float>(2) = 0;
		position_filter_.statePost.at<float>(3) = 0;
		position_filter_.statePost.copyTo(position_filter_.statePre);

		//dimensions
		dimension_filter_.statePost.at<float>(0) = measurement.width;
		dimension_filter_.statePost.at<float>(1) = measurement.height;
		dimension_filter_.statePost.at<float>(2) = 0;
		dimension_filter_.statePost.at<float>(3) = 0;
		dimension_filter_.statePost.copyTo(dimension_filter_.statePre);
}

void Kalman::update(const cv::Rect* measurement) {
	//prediction step
		//position
		location_estimate_ = position_filter_.predict();
		//dimensions
		dimension_estimate_ = dimension_filter_.predict();

	//correction step
		if(measurement!=NULL) {
			//position
			cv::Mat position_measurement_mat(2,1,CV_32FC1);
			position_measurement_mat.at<float>(0)=measurement->x+measurement->width/2;
			position_measurement_mat.at<float>(1)=measurement->y+measurement->height/2;
			position_filter_.correct(position_measurement_mat);
			//dimensions
			cv::Mat dimension_measurement_mat(2,1,CV_32FC1);
			dimension_measurement_mat.at<float>(0)=measurement->width;
			dimension_measurement_mat.at<float>(1)=measurement->height;
			dimension_filter_.correct(dimension_measurement_mat);
		}

	//re-store that information as a cv::Rect for convinience
	float x = location_estimate_.at<float>(0)-dimension_estimate_.at<float>(0)/2,
		  y = location_estimate_.at<float>(1)-dimension_estimate_.at<float>(1)/2,
		  width = dimension_estimate_.at<float>(0),
		  height = dimension_estimate_.at<float>(1);

	bounding_rect_ = cv::Rect(x,y,width,height);
}
