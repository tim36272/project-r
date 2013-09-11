#include <ros/ros.h>
#include "MessageFetcher.h"

static const std::string kDepthWindow = "depth";


int main(int argc, char** argv)
{
	cv::namedWindow(kDepthWindow,CV_WINDOW_OPENGL);
	cv::moveWindow(kDepthWindow,1000,20);

	ros::init(argc, argv, "depth");

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

		//show the image on screen
		imshow(kDepthWindow,depth_raw);

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
