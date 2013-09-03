#include "Analysis.h"

#define R_UPPER_COLOR 0
#define R_LOWER_COLOR 1

Analysis::Analysis() {
	frame_number_ = 0;
}

Analysis::Analysis(const std::string& people_topic, const std::string& bags_topic){

	//subscribe to color topics
	person_subscription_ = node_handle_.subscribe(people_topic, 10, &Analysis::PersonColorCallback,this);
	bag_subscription_ = node_handle_.subscribe(bags_topic, 10, &Analysis::BagColorCallback,this);
	frame_number_ = 0;
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
	std::cout<<"Added a person"<<std::endl;
}
void Analysis::BagColorCallback(const dhs::bag& msg) {
	cv::Scalar temp;
	temp[0] = msg.blue;
	temp[0] = msg.green;
	temp[0] = msg.red;
	bag_colors_.push_back(temp);
	std::cout<<"Added a bag"<<std::endl;
}


int Analysis::Update(const cv::Mat& color_raw, const cv::Mat& depth_raw, PersonList* known_people, BagList* known_bags) {
	frame_number_++;

	bool debug = false ;

	cv::Mat hsv_raw;
	cv::cvtColor(color_raw,hsv_raw,CV_BGR2HSV);
if(debug) ROS_INFO_STREAM("Passed step 1");
	ScanForNewPeople(color_raw,hsv_raw,depth_raw,known_people);
if(debug) ROS_INFO_STREAM("Passed step 2");
	ScanForNewBags(color_raw,hsv_raw,depth_raw,*known_people,known_bags);
if(debug) ROS_INFO_STREAM("Passed step 3");
	utility::UpdateKalmans(frame_number_,known_people);
if(debug) ROS_INFO_STREAM("Passed step 4");
	utility::UpdateKalmans(frame_number_,known_bags);
if(debug) ROS_INFO_STREAM("Passed step 5");
	utility::CullList(frame_number_,known_people);
if(debug) ROS_INFO_STREAM("Passed step 6");
	utility::CullList(frame_number_,known_bags);

	return frame_number_;
}
void Analysis::ScanForNewPeople(const cv::Mat& color_raw, const cv::Mat& hsv_raw, const cv::Mat& depth_raw, PersonList* known_people) {
	//get a map of each color
	cv::Mat upper_projection,lower_projection;
	for(uint color_index=0;color_index<colors_.size();color_index++) {
		utility::GetSpecificColorMap(hsv_raw,colors_[color_index].values[R_UPPER_COLOR],&upper_projection);
		utility::GetSpecificColorMap(hsv_raw,colors_[color_index].values[R_LOWER_COLOR],&lower_projection);
		//get regions in both projections
			//get a list of possible locations
			std::vector<cv::Rect> candidate_areas;
			for(uint person_index=0;person_index<known_people->size();person_index++) {
				candidate_areas.push_back((*known_people)[person_index].filter.bounding_rect());
			}
		RegionList upper_regions,lower_regions;
		utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,upper_projection,&upper_regions);
		utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,lower_projection,&lower_regions);


		//check if another blob has ownership of this color
		std::vector<int> blobs_using_this_color;
		utility::CheckForPersonUsingColorScheme(*known_people,colors_[color_index].values[R_UPPER_COLOR],colors_[color_index].values[R_LOWER_COLOR],&blobs_using_this_color);
		if(blobs_using_this_color.size()!=0) {
			//update those regions
			for(uint blob_index=0;blob_index<blobs_using_this_color.size();blob_index++) {
				utility::Pair regions_to_delete = utility::TryToUpdatePerson(frame_number_,upper_regions,lower_regions,&(*known_people)[blobs_using_this_color[blob_index]]);
				if(!regions_to_delete.Empty()) {
					std::cout<<"Found person "<<blob_index<<" again"<<std::endl;
					utility::DeleteRegion(regions_to_delete.values[0],&upper_regions);
					utility::DeleteRegion(regions_to_delete.values[1],&lower_regions);
				}
			}
		}
		//everything left is a region we haven't added to known_blobs yet
		else if((upper_regions.size()!=0 && lower_regions.size()!=0)) {
			//check for vertical regions
			for(uint upper_index=0;upper_index<upper_regions.size();upper_index++) {
				for(uint lower_index=0;lower_index<lower_regions.size();lower_index++) {
					if(utility::AreVerticallyAligned(upper_regions[upper_index], lower_regions[lower_index])) {
						std::cout<<"adding a person"<<std::endl;
						PersonDescriptor person;
						utility::NewPerson(upper_regions[upper_index], lower_regions[lower_index],colors_[color_index],frame_number_,&person);
						known_people->push_back(person);
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
		utility::GetSpecificColorMap(hsv_raw,bag_colors_[color_index],&projection);
		//get regions in both projections
			//get a list of possible locations
			std::vector<cv::Rect> candidate_areas;
			for(uint bag_index=0;bag_index<known_bags->size();bag_index++) {
				candidate_areas.push_back((*known_bags)[bag_index].filter.bounding_rect());
			}
		RegionList regions;
		utility::GetSignificantRegions(hsv_raw.size(),candidate_areas,projection,&regions);

		//check if another blob has ownership of this color
		std::vector<int> blobs_using_this_color;
		utility::CheckForBagUsingColorScheme(*known_bags,bag_colors_[color_index],&blobs_using_this_color);
		if(blobs_using_this_color.size()!=0) {
			//update those regions
			for(uint blob_index=0;blob_index<blobs_using_this_color.size();blob_index++) {
				int owner_index= (*known_bags)[blobs_using_this_color[blob_index]].belongs_to;
				cv::Rect owner_position = known_people[owner_index].filter.bounding_rect();
				int region_to_delete = utility::TryToUpdateBag(frame_number_,regions,owner_position,&(*known_bags)[blobs_using_this_color[blob_index]]);
				if(region_to_delete!=-1) {
					std::cout<<"Found bag "<<blob_index<<" again"<<std::endl;
					utility::DeleteRegion(region_to_delete,&regions);
				}
			}
		}
		//everything left is a region we haven't added to known_bags yet
		else if(regions.size()!=0) {
			bool bag_added = false;
			for(uint region_index=0;region_index<regions.size();region_index++) {
				//check for regions in the search window for each person
				for(uint person_index=0;person_index<known_people.size();person_index++) {
					cv::Rect owner_location = known_people[person_index].filter.bounding_rect();
					cv::Rect search_window(owner_location.x-owner_location.width,
										   owner_location.y+owner_location.height/2,
										   owner_location.width*3,
										   owner_location.height/2);
					if( utility::RectsOverlap(search_window,boundingRect(regions[region_index])) ) {
						//add this bag to the owner
						BagDescriptor bag;
						utility::NewBag(regions[region_index],bag_colors_[color_index],frame_number_,person_index,&bag);
						known_bags->push_back(bag);
						bag_added=true;
					}
				}
			}
			if(!bag_added) {
				//add orphan bags here
				int dummy;
			}
		}
	}
}

