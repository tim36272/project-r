#include <ros/ros.h>
#include "MessageFetcher.h"

static const std::string kHSVWindow = "depth";


int main(int argc, char** argv)
{
	cv::namedWindow(kHSVWindow,CV_WINDOW_OPENGL);

	ros::init(argc, argv, "hsv");

	MessageFetcher ros_handle;
	cv::Mat color_raw,depth_raw,hsv_raw;
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

		cv::cvtColor(color_raw,hsv_raw,CV_BGR2HSV);
		//show the image on screen
		imshow(kHSVWindow,hsv_raw);

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
