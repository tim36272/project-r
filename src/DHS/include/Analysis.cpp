#include "Analysis.h"

#define R_UPPER_COLOR 0
#define R_LOWER_COLOR 1
#define NO_OWNER -1
#define KERNAL 15

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
}

void Analysis::dump() {
	colors_.clear();
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
	colors_.push_back(temp);
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
	cv::Mat color_internal,hsv_raw,hsv_masked,depth_masked,color_temp;
	bool debug = false;
	cv::cvtColor(color_raw,hsv_raw,CV_BGR2HSV);

	cv::GaussianBlur(color_raw,color_temp,cv::Size(KERNAL,KERNAL),0);

	utility::Normalize(frame_number_,hsv_raw,&color_temp,&last_mean_);
																if(debug) ROS_INFO_STREAM("Passed step 1");
	GetHint(color_temp,depth_raw,&hint);
																if(debug) ROS_INFO_STREAM("Passed step 2");
	ApplyHint(hint,depth_raw,color_temp,&color_internal,&depth_masked);
																if(debug) ROS_INFO_STREAM("Passed step 3");
	imshow("hinted",color_internal);
	cv::cvtColor(color_internal,hsv_masked,CV_BGR2HSV);
																if(debug) ROS_INFO_STREAM("Passed step 4");
	ScanForNewPeople(color_internal,hsv_masked,depth_masked,known_people);
																if(debug) ROS_INFO_STREAM("Passed step 5");
	ScanForNewBags(color_internal,hsv_masked,depth_masked,*known_people,known_bags);
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
	//process the incoming frames
	//blur
	cv::Mat depth_thresholded,color_foreground,	depth_foreground;
	cv::threshold(depth_raw,depth_thresholded,250,255,CV_THRESH_TOZERO_INV);
	//add to segmentation
	color_segmentation_.operator ()(color_raw,color_foreground);
	depth_segmentation_.operator ()(depth_thresholded,depth_foreground);

	imshow("color_foreground",color_foreground);
	imshow("depth_foreground",depth_foreground);
	//erode/dilate foregrounds
	int iterations=3;
	cv::dilate(color_foreground,color_foreground,cv::Mat(),cv::Point(-1,-1),iterations);
	cv::dilate(depth_foreground,depth_foreground,cv::Mat(),cv::Point(-1,-1),iterations);
	cv::erode(color_foreground,color_foreground,cv::Mat(),cv::Point(-1,-1),iterations);
	cv::erode(depth_foreground,depth_foreground,cv::Mat(),cv::Point(-1,-1),iterations);


	//threshold the shadows out of the color image
	cv::threshold(color_foreground,color_foreground,254,0,CV_THRESH_TOZERO);

	*hint = depth_foreground | color_foreground;
}

void Analysis::ApplyHint(const cv::Mat& hint, const cv::Mat& depth_raw, const cv::Mat& color_raw, cv::Mat* color_masked, cv::Mat* depth_masked) {
	color_raw.copyTo(*color_masked,hint);
	depth_raw.copyTo(*depth_masked,hint);

}

void Analysis::ScanForNewPeople(const cv::Mat& color_raw, const cv::Mat& hsv_raw, const cv::Mat& depth_raw, PersonList* known_people) {
	//get a map of each color
	cv::Mat upper_projection,lower_projection;
	for(uint color_index=0;color_index<colors_.size();color_index++) {
		utility::GetSpecificColorMap(hsv_raw,colors_[color_index][R_UPPER_COLOR],&upper_projection);
		utility::GetSpecificColorMap(hsv_raw,colors_[color_index][R_LOWER_COLOR],&lower_projection);

		//check if another blob has ownership of this color
		std::vector<int> blobs_using_this_color;
		utility::CheckForPersonUsingColorScheme(*known_people,colors_[color_index][R_UPPER_COLOR],colors_[color_index][R_LOWER_COLOR],&blobs_using_this_color);
		if(blobs_using_this_color.size()!=0) {
			//update those regions
			for(uint blob_index=0;blob_index<blobs_using_this_color.size();blob_index++) {
				//get regions in both projections
					//get a list of possible locations
					std::vector<cv::Rect> candidate_areas;
					candidate_areas.push_back((*known_people)[blobs_using_this_color[blob_index]].filter.bounding_rect());
				RegionList upper_regions,lower_regions;
				utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,upper_projection,&upper_regions);
				utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,lower_projection,&lower_regions);


				utility::Pair regions_to_delete = utility::TryToUpdatePerson(depth_raw,frame_number_,upper_regions,lower_regions,&(*known_people)[blobs_using_this_color[blob_index]]);
				if(!regions_to_delete.Empty()) {
					//erase those regions from the original color maps
					cv::Mat upper_erasure_mask(color_raw.size(),CV_8UC1,cv::Scalar(0)),
							lower_erasure_mask(color_raw.size(),CV_8UC1,cv::Scalar(0));
					cv::drawContours(upper_erasure_mask,upper_regions,regions_to_delete[0],cv::Scalar(255),-1,8);
					cv::drawContours(lower_erasure_mask,lower_regions,regions_to_delete[1],cv::Scalar(255),-1,8);
					//erase
					upper_projection = upper_projection - upper_erasure_mask;
					lower_projection = lower_projection - lower_erasure_mask;

				}
			}
		}
		//everything left is a region we haven't added to known_blobs yet
		else{
			//get regions in both projections
				//get a list of possible locations
				std::vector<cv::Rect> candidate_areas;
				for(uint person_index=0;person_index<known_people->size();person_index++) {
					candidate_areas.push_back((*known_people)[person_index].filter.bounding_rect());
				}
			RegionList upper_regions,lower_regions;
			utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,upper_projection,&upper_regions);
			utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,lower_projection,&lower_regions);

			//check for vertically aligned regions
			for(uint upper_index=0;upper_index<upper_regions.size();upper_index++) {
				for(uint lower_index=0;lower_index<lower_regions.size();lower_index++) {
					if(utility::AreVerticallyAlignedOriented(upper_regions[upper_index], lower_regions[lower_index])) {
						std::cout<<"adding a person"<<std::endl;
						PersonDescriptor person;
						utility::NewPerson(depth_raw,upper_regions[upper_index], lower_regions[lower_index],colors_[color_index],frame_number_,&person);

						std::cout<<"added person"<<std::endl;
						known_people->push_back(person);
						std::cout<<"pushed person"<<std::endl;
					}
				}
			}
		}
	}
}

void Analysis::ScanForNewBags(const cv::Mat& color_raw, const cv::Mat& hsv_raw, const cv::Mat& depth_raw, const PersonList& known_people, BagList* known_bags) {
	//get a map of each color
	cv::Mat projection;
	for(uint color_index=0;color_index<bag_colors_.size();color_index++) {
		std::cout<<"TS_CP0.01"<<std::endl;
		utility::GetSpecificColorMap(hsv_raw,bag_colors_[color_index],&projection);
		std::cout<<"TS_CP0.1"<<std::endl;
		//check if another blob has ownership of this color
		std::vector<int> blobs_using_this_color;
		utility::CheckForBagUsingColorScheme(*known_bags,bag_colors_[color_index],&blobs_using_this_color);
		std::cout<<"TS_CP0.2"<<std::endl;
		if(blobs_using_this_color.size()!=0) {
			//update those regions
			for(uint blob_index=0;blob_index<blobs_using_this_color.size();blob_index++) {
				std::cout<<"TS_CP1"<<std::endl;
				//get hint of where the the bag might be based on the owner
				int owner_index= (*known_bags)[blobs_using_this_color[blob_index]].belongs_to;
				cv::Rect owner_position;
				if(owner_index!=NO_OWNER && (int(known_people.size())-1)>=owner_index) owner_position = known_people[owner_index].filter.bounding_rect();
				std::cout<<"TS_CP2"<<std::endl;
				//get regions in projection
					//get a list of possible locations
					std::vector<cv::Rect> candidate_areas;
					for(uint bag_index=0;bag_index<known_bags->size();bag_index++) {
						candidate_areas.push_back((*known_bags)[bag_index].filter.bounding_rect());
					}
					std::cout<<"TS_CP3"<<std::endl;
					std::cout<<"TS_CP4"<<std::endl;
				RegionList regions;
				utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,projection,&regions);
				std::cout<<"TS_CP5"<<std::endl;
				int region_to_delete = utility::TryToUpdateBag(depth_raw,frame_number_,regions,owner_position,&(*known_bags)[blobs_using_this_color[blob_index]]);
				std::cout<<"TS_CP6"<<std::endl;
				if(region_to_delete!=-1) {
					//erase those regions from the original color maps
					cv::Mat erasure_mask(color_raw.size(),CV_8UC1,cv::Scalar(0));
					cv::drawContours(erasure_mask,regions,region_to_delete,cv::Scalar(255),-1,8);
					//erase
					projection = projection - erasure_mask;

				}
				std::cout<<"TS_CP7"<<std::endl;
			}
		}
		//everything left is a region we haven't added to known_bags yet
		else{
			std::cout<<"TS_2CP1"<<std::endl;
			//get regions in both projections
				//get a list of possible locations
				std::vector<cv::Rect> candidate_areas;
				for(uint bag_index=0;bag_index<known_bags->size();bag_index++) {
					candidate_areas.push_back((*known_bags)[bag_index].filter.bounding_rect());
				}
				std::cout<<"TS_2CP2"<<std::endl;
			RegionList regions;
			utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,projection,&regions);
			std::cout<<"TS_2CP3"<<std::endl;

			bool bag_added = false;
			for(uint region_index=0;region_index<regions.size();region_index++) {
				std::cout<<"TS_2CP4"<<std::endl;
				//check for regions in the search window for each person
				for(uint person_index=0;person_index<known_people.size();person_index++) {
					cv::Rect owner_location = known_people[person_index].filter.bounding_rect();
					cv::Rect search_window(owner_location.x-owner_location.width,
										   owner_location.y+owner_location.height/2,
										   owner_location.width*3,
										   owner_location.height/2);
					std::cout<<"TS_2CP5"<<std::endl;
					if( utility::RectsOverlap(search_window,boundingRect(regions[region_index])) ) {
						//add this bag to the owner
						BagDescriptor bag;
						std::cout<<"TS_2CP6"<<std::endl;
						utility::NewBag(depth_raw,regions[region_index],bag_colors_[color_index],frame_number_,person_index,&bag);
						std::cout<<"TS_2CP7"<<std::endl;
						known_bags->push_back(bag);
						std::cout<<"TS_2CP8"<<std::endl;
						bag_added=true;
					}
				}
			}
			if(!bag_added && regions.size()>0) {
				//add orphan bags here
				std::cout<<"TS_2CP9"<<std::endl;
				BagDescriptor bag;

				utility::NewBag(depth_raw,regions[0],bag_colors_[color_index],frame_number_,0,&bag);
				std::cout<<"TS_2CP10"<<std::endl;
				known_bags->push_back(bag);
				std::cout<<"TS_2CP11"<<std::endl;
			}
		}
	}
}

