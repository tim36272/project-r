#include "Analysis.h"

#define R_UPPER_COLOR 0
#define R_LOWER_COLOR 1
#define NO_OWNER -1
#define KERNAL 15

static const int kMinRegionSize=3000;

Analysis::Analysis() {
	assert(false);
	frame_number_ = 0;
}

Analysis::Analysis(const std::string& people_topic, const std::string& bags_topic){

	//subscribe to color topics
	person_subscription_ = node_handle_.subscribe(people_topic, 10, &Analysis::PersonColorCallback,this);
	bag_subscription_ = node_handle_.subscribe(bags_topic, 10, &Analysis::BagColorCallback,this);
	frame_number_ = 0;

	color_segmentation_.set("nmixtures",3);
	depth_segmentation_.set("nmixtures",16);
	color_segmentation_.set("history",1000);
	depth_segmentation_.set("history",1000);
	depth_segmentation_.set("detectShadows",0);
	color_segmentation_.set("detectShadows",1);

	depth_segmentation_.set("fVarInit",1);
	color_segmentation_.set("fVarInit",1);

	depth_segmentation_.set("fVarMax",1);
	color_segmentation_.set("fVarMax",1);

	depth_mats_averaged_ = 0;
}

void Analysis::dump() {
	people_colors_.clear();
	bag_colors_.clear();
}
void Analysis::PersonColorCallback(const dhs::person& msg) {
	utility::ColorPair temp;
	temp[0][0] = msg.upper_blue;
	temp[0][1] = msg.upper_green;
	temp[0][2] = msg.upper_red;

	temp[1][0] = msg.lower_blue;
	temp[1][1] = msg.lower_green;
	temp[1][2] = msg.lower_red;
	people_colors_.push_back(temp);
	std::cout<<"Analysis added a person"<<std::endl;
}
void Analysis::BagColorCallback(const dhs::bag& msg) {
	cv::Scalar temp;
	temp[0] = msg.blue;
	temp[1] = msg.green;
	temp[2] = msg.red;
	bag_colors_.push_back(temp);
	std::cout<<"Analysis added a bag"<<std::endl;
}


int Analysis::Update(const cv::Mat& color_raw, const cv::Mat& depth_raw, PersonList* known_people, BagList* known_bags) {
	frame_number_++;

	cv::Mat hint;
	cv::Mat color_masked,hsv_raw,hsv_masked,depth_masked,color_temp;
	RegionList hint_regions;
	bool debug = false;
	cv::cvtColor(color_raw,hsv_raw,CV_BGR2HSV);

	cv::GaussianBlur(color_raw,color_temp,cv::Size(KERNAL,KERNAL),0);

	utility::Normalize(frame_number_,hsv_raw,&color_temp,&last_mean_);
																if(debug) ROS_INFO_STREAM("Passed step 1");
	GetHint(color_temp,depth_raw,&hint);
																if(debug) ROS_INFO_STREAM("Passed step 2");
	ApplyHint(hint,depth_raw,color_temp,&color_masked,&depth_masked);
																if(debug) ROS_INFO_STREAM("Passed step 3");
	imshow("hinted",color_masked);
	cv::cvtColor(color_masked,hsv_masked,CV_BGR2HSV);
																if(debug) ROS_INFO_STREAM("Passed step 4");
	ScanForPeople(hint,color_masked,hsv_masked,depth_masked,known_people);
																if(debug) ROS_INFO_STREAM("Passed step 5");
	ScanForBags(hint,color_masked,hsv_masked,depth_masked,known_bags);
																if(debug) ROS_INFO_STREAM("Passed step 6");
	utility::UpdateKalmans(frame_number_,known_people);
																if(debug) ROS_INFO_STREAM("Passed step 7");
	utility::UpdateKalmans(frame_number_,known_bags);
																if(debug) ROS_INFO_STREAM("Passed step 8");
	utility::CullList(frame_number_,known_people);
																if(debug) ROS_INFO_STREAM("Passed step 9");
	utility::CullList(frame_number_,known_bags);
																if(debug) ROS_INFO_STREAM("Passed step 10");

	return frame_number_;
}

void Analysis::GetHint(const cv::Mat& color_raw, const cv::Mat& depth_raw, cv::Mat* hint) {
	//process use the depth threshold
	int depth_threshold = 150;
	//int depth_threshold = 110;
	cv::threshold(depth_raw,*hint,depth_threshold,255,CV_THRESH_TOZERO_INV);
	RegionList regions;

	int iterations = 5;
	cv::erode(*hint,*hint,cv::Mat(),cv::Point(-1,-1),iterations);
	cv::dilate(*hint,*hint,cv::Mat(),cv::Point(-1,-1),iterations);

	cv::findContours(*hint,regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	//remove small contours
	for(int i=regions.size()-1;i>=0;i--) {
		if(cv::contourArea(regions[i]) < 4000) {
			regions.erase(regions.begin() + i);
		}
	}

	cv::Mat temp(hint->size(),CV_8UC1,cv::Scalar(0));

	cv::drawContours(temp,regions,-1,cv::Scalar(255),-1,4);

	*hint = temp;

}

void Analysis::ApplyHint(const cv::Mat& hint, const cv::Mat& depth_raw, const cv::Mat& color_raw, cv::Mat* color_masked, cv::Mat* depth_masked) {
	color_raw.copyTo(*color_masked,hint);
	depth_raw.copyTo(*depth_masked,hint);

	//remove skin tones
	cv::Mat hsv_masked;
	cv::cvtColor(*color_masked,hsv_masked,CV_BGR2HSV);

	std::vector<cv::Mat> planes;
	cv::split(hsv_masked,planes);

	cv::threshold(planes[0],planes[0],15,0,CV_THRESH_TOZERO_INV);
	cv::threshold(planes[0],planes[0],0,0,CV_THRESH_TOZERO);
	cv::threshold(planes[0],planes[0],1,255,CV_THRESH_BINARY_INV);

//	utility::MaskToZero(planes[0],color_masked);
}

void Analysis::ScanForPeople(cv::Mat& hint,const cv::Mat& color_masked, const cv::Mat& hsv_masked, const cv::Mat& depth_masked, PersonList* known_people) {
	//for each known person, segment the masks according to their bounding box
	for(uint color_index=0;color_index<people_colors_.size();color_index++) {
		int person_using_color = utility::CheckForPersonUsingColorScheme(*known_people,people_colors_[color_index][0],people_colors_[color_index][1]);
		//see if they are visible
		//get blobs in color mask
		RegionList regions;
		cv::findContours(hint,regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); //guaranteed to be large regions
		//for look at each region's top and bottom and see if it matches the color scheme
		for(uint region_index=0;region_index<regions.size();region_index++) {
			//make a mask of just the top
			cv::Rect outside_bounding_rect = cv::boundingRect(regions[region_index]);
			if(outside_bounding_rect.y > color_masked.rows/2) continue; // the blob is too short
			cv::Rect upper = utility::UpperPersonHalf(outside_bounding_rect);
			cv::Rect lower = utility::LowerPersonHalf(outside_bounding_rect);

			cv::Mat color_upper_mask(color_masked.size(),CV_8UC1,cv::Scalar(0)),color_lower_mask(color_masked.size(),CV_8UC1,cv::Scalar(0));
			cv::rectangle(color_upper_mask,upper,cv::Scalar(255),-1,8);
			cv::rectangle(color_lower_mask,lower,cv::Scalar(255),-1,8);

			//see if this could be the color
			cv::Mat upper_hsv_projection(color_masked.size(),CV_8UC1,cv::Scalar(0)),lower_hsv_projection(color_masked.size(),CV_8UC1,cv::Scalar(0));
			hsv_masked.copyTo(upper_hsv_projection,color_upper_mask);
			hsv_masked.copyTo(lower_hsv_projection,color_lower_mask);

			//get region sizes here to compare to later
			RegionList saved_regions;
			cv::Mat temp_saved;
			cv::cvtColor(upper_hsv_projection,temp_saved,CV_BGR2GRAY);

			cv::findContours(temp_saved,saved_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int saved_max=-1,saved_max_at=-1;
			for(int i=0;i<saved_regions.size();i++) {
				if(cv::contourArea(saved_regions[i]) > saved_max) {
					saved_max = cv::contourArea(saved_regions[i]);
					saved_max_at = i;
				}
			}

			int upper_max_comparable;
			if(saved_max_at!=-1) upper_max_comparable = cv::contourArea(saved_regions[saved_max_at]);
			else continue; // got nothing in the upper whole region


			std::vector<cv::Mat> upper_planes,lower_planes;

			cv::split(upper_hsv_projection,upper_planes);
			cv::split(lower_hsv_projection,lower_planes);

			int target_hue = people_colors_[color_index][0][0];
			int target_sat = people_colors_[color_index][0][1];
			int hue_window = 8;
			int sat_window = 75;
			cv::threshold(upper_planes[0],upper_planes[0],target_hue-hue_window,255,CV_THRESH_TOZERO);
			cv::threshold(upper_planes[0],upper_planes[0],target_hue+hue_window,255,CV_THRESH_TOZERO_INV);

			cv::threshold(upper_planes[1],upper_planes[1],target_sat-sat_window,255,CV_THRESH_TOZERO);
			cv::threshold(upper_planes[1],upper_planes[1],target_sat+sat_window,255,CV_THRESH_TOZERO_INV);


			target_hue = people_colors_[color_index][1][0];
			cv::threshold(lower_planes[0],lower_planes[0],target_hue-hue_window,255,CV_THRESH_TOZERO);
			cv::threshold(lower_planes[0],lower_planes[0],target_hue+hue_window,255,CV_THRESH_TOZERO_INV);

			upper_planes[0] = upper_planes[0] & upper_planes[1];


			//find the area occupied by this color
			RegionList this_upper_projection;
			cv::findContours(upper_planes[0],this_upper_projection,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int upper_max_size=0;
			int upper_max_at=-1;
			if(this_upper_projection.size()>0) {
				for(uint i=0;i<this_upper_projection.size();i++) {
					int region_size = cv::contourArea(this_upper_projection[i]);
						upper_max_size += region_size;
						upper_max_at = i;
				}
			}
			else continue; //this couldn't be it because there are 0 things this color

			RegionList this_lower_projection;
			cv::findContours(lower_planes[0],this_lower_projection,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int lower_max_size=kMinRegionSize;
			int lower_max_at=-1;
			if(this_lower_projection.size()>0) {
				for(uint i=0;i<this_lower_projection.size();i++) {
					int region_size = cv::contourArea(this_lower_projection[i]);
					if(region_size > lower_max_size) {
						lower_max_size = region_size;
						lower_max_at = i;
					}
				}
			}
			else continue; //this couldn't be it



//			std::cout<<"Upper max region size: "<<upper_max_size<<std::endl;
//			std::cout<<"lower max region size: "<<lower_max_size<<std::endl;

//			std::cout<<"Whole region size: "<<cv::contourArea(regions[region_index])<<std::endl;

			//this is the blob if the combined region size is at least 25% of the original blob
//			std::cout<<"comparing "<<upper_max_size<<" to "<<upper_max_comparable<<std::endl;
			if( (upper_max_size) > /*upper_max_comparable*0.25*/ 1000) {
				//this is probably the same person
				if(person_using_color==-1) { //new person
					PersonDescriptor temp_person;
					utility::NewPerson(depth_masked,upper,lower,people_colors_[color_index],frame_number_,&temp_person);
					known_people->push_back(temp_person);
				}
				else {
					//existing person
					utility::UpdatePerson(depth_masked,cv::Scalar(0),cv::Scalar(0),frame_number_,utility::UpperPersonHalf(cv::boundingRect(regions[region_index])),utility::LowerPersonHalf(cv::boundingRect(regions[region_index])),&((*known_people)[person_using_color]));
				}
			}
			//otherwise this is not the human we are looking for
		}
	}
}

void Analysis::ScanForBags(const cv::Mat& hint,const cv::Mat& color_masked, const cv::Mat& hsv_masked, const cv::Mat& depth_masked, BagList* known_bags) {
	//for each known person, segment the masks according to their bounding box
	for(uint color_index=0;color_index<bag_colors_.size();color_index++) {
		int bag_using_color = utility::CheckForBagUsingColorScheme(*known_bags,bag_colors_[color_index]);
		//see if they are visible
		//get blobs in color mask
		RegionList regions;
		cv::findContours(hint,regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); //guaranteed to be large regions
		//for look at each region's top and bottom and see if it matches the color scheme
		for(uint region_index=0;region_index<regions.size();region_index++) {
			//check if it is too tall

			//make a mask of just the top
			cv::Rect outside_bounding_rect = cv::boundingRect(regions[region_index]);
			if(outside_bounding_rect.y < color_masked.rows/2) continue; // the blob is too tall
			cv::Rect upper = outside_bounding_rect;

			cv::Mat color_upper_mask(color_masked.size(),CV_8UC1,cv::Scalar(0));
			cv::rectangle(color_upper_mask,upper,cv::Scalar(255),-1,8);

			//see if this could be the color
			cv::Mat upper_hsv_projection(color_masked.size(),CV_8UC1,cv::Scalar(0));
			hsv_masked.copyTo(upper_hsv_projection,color_upper_mask);

			//get region sizes here to compare to later
			RegionList saved_regions;
			cv::Mat temp_saved;
			cv::cvtColor(upper_hsv_projection,temp_saved,CV_BGR2GRAY);

			cv::findContours(temp_saved,saved_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int saved_max=-1,saved_max_at=-1;
			for(int i=0;i<saved_regions.size();i++) {
				if(cv::contourArea(saved_regions[i]) > saved_max) {
					saved_max = cv::contourArea(saved_regions[i]);
					saved_max_at = i;
				}
			}

			int upper_max_comparable;
			if(saved_max_at!=-1) upper_max_comparable = cv::contourArea(saved_regions[saved_max_at]);
			else continue; // got nothing in the upper whole region


			std::vector<cv::Mat> upper_planes;

			cv::split(upper_hsv_projection,upper_planes);

			int target_hue = bag_colors_[color_index][0];
			int hue_window = 7;
			cv::threshold(upper_planes[0],upper_planes[0],target_hue-hue_window,255,CV_THRESH_TOZERO);
			cv::threshold(upper_planes[0],upper_planes[0],target_hue+hue_window,255,CV_THRESH_TOZERO_INV);

			//find the area occupied by this color
			RegionList this_upper_projection;
			cv::findContours(upper_planes[0],this_upper_projection,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			int upper_max_size=0;
			int upper_max_at=-1;
			if(this_upper_projection.size()>0) {
				for(uint i=0;i<this_upper_projection.size();i++) {
					int region_size = cv::contourArea(this_upper_projection[i]);
						upper_max_size += region_size;
						upper_max_at = i;
				}
			}
			else continue; //this couldn't be it



//			std::cout<<"Upper max region size: "<<upper_max_size<<std::endl;

//			std::cout<<"Whole region size: "<<cv::contourArea(regions[region_index])<<std::endl;

			//this is the blob if the combined region size is at least 25% of the original blob
//			std::cout<<"comparing "<<upper_max_size<<" to "<<upper_max_comparable<<std::endl;
			if( (upper_max_size) > upper_max_comparable*0.50) {
				//this is probably the same person
				if(bag_using_color==-1) {
					BagDescriptor temp_bag;
					utility::NewBag(depth_masked,upper,bag_colors_[color_index],frame_number_,&temp_bag);
					known_bags->push_back(temp_bag);
				}
				else {
					utility::UpdateBag(depth_masked,cv::Scalar(0),frame_number_,upper,&(*known_bags)[bag_using_color]);
			}
			//otherwise this is not the human we are looking for

			}
		}
	}
}

