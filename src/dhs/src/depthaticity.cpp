#include <ros/ros.h>
#include "MessageFetcher.h"

static const std::string kHSVWindow = "Depthaticity";


int main(int argc, char** argv)
{
	cv::namedWindow(kHSVWindow,CV_WINDOW_OPENGL);

	ros::init(argc, argv, "depthaticity");

	MessageFetcher ros_handle;
	cv::Mat color_raw,depth_raw;
	bool run = true;

	//main loop
	while(ros::ok() && run) {
		//spin until we get a new frame
		if(!ros_handle.GetFrame(color_raw,depth_raw)) {
			ros::spinOnce();
			if(cv::waitKey(1)=='q') {
				run=false;
			}
			continue;
		}
		std::vector<cv::Mat> planes,depthaticity_planes(3);
		cv::Mat depthaticity;
		std::cout<<"TS_CP0"<<std::endl;
		cv::split(color_raw,planes);
std::cout<<"TS_CP1"<<std::endl;
		//
		cv::Mat denominator(color_raw.size(),CV_8UC1);
		std::cout<<"TS_CP1.1"<<std::endl;
		planes[0].convertTo(planes[0],CV_8U);
		planes[1].convertTo(planes[1],CV_8U);
		planes[2].convertTo(planes[2],CV_8U);
		depth_raw.convertTo(depth_raw,CV_8U,255./2000.);

		denominator = planes[0]/4.+planes[1]/4.+planes[2]/4.+depth_raw/4.;
imshow("deonminator",denominator);
		std::cout<<"TS_CP1"<<std::endl;
		imshow("planes 0 before",planes[0]);
		cv::divide(planes[0],denominator,depthaticity_planes[0]);
		imshow("planes 0 after",depthaticity_planes[0]);
		planes[1] /=denominator;
		planes[2] /=denominator;
		std::cout<<"TS_CP2"<<std::endl;
		cv::merge(planes,depthaticity);
		std::cout<<"TS_CP3"<<std::endl;
		//show the image on screen
		imshow(kHSVWindow,depthaticity);
		std::cout<<"TS_CP4"<<std::endl;
		char key = cv::waitKey(1);
		switch(key) {
		case 'q':
			run = false;
			break;
		}
	}
	//cleanup
	cv::destroyAllWindows();

	return 0;
}
