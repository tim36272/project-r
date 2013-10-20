#ifndef TIM_SRC_UTILITIES_CPP
#define TIM_SRC_UTILITIES_CPP

#include "utility.h"

//histogram calculation
#define R_UPPER_COLOR 0
#define R_LOWER_COLOR 1
#define R_RED_WRAP_WINDOW 15

//region splitting constants
static const int kMaxRegionSplitTimes = 10;
static const int kMaxRegionCombineTimes = 5;

static const int kDepthClosenessThreshold = 40;

static const int kMinRegionSize=2000;
static const int kMaxRegionSize=18000;



namespace utility {


	cv::Point calcCenter(const cv::Rect& rectangle) {
		cv::Point center(rectangle.x+rectangle.width/2,rectangle.y+rectangle.height/2);
		return center;
	}
	double distance(cv::Point lhs, cv::Point rhs) {
		return sqrt(pow(lhs.x-rhs.x,2)+pow(lhs.y-rhs.y,2));
	}
	/*
	 * Name: SplitRegion
	 * Description: divides a region if there is a weakly connected segment
	 */
	void SplitRegion(const cv::Point& mass_center, cv::Mat& region, RegionList* new_blobs) {
		RegionList temp_regions;
		int timeout=0;

		//erode the region a maximum of kMaxRegionSplitTimes
		while(temp_regions.size()<2 && timeout <kMaxRegionSplitTimes) {
			temp_regions.clear();
			cv::Mat temp_region;
			//erode the region and refind blobs in hopes of getting two or more blobs
			cv::erode(region,region,cv::Mat());
			region.copyTo(temp_region);
			cv::findContours(temp_region,temp_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			timeout++;
		}
		//check if the regions are vertical
		for(int first_region_index=0;first_region_index<temp_regions.size();first_region_index++) {
			for(int second_region_index=0;second_region_index<temp_regions.size();second_region_index++) {
				//check every region and if a lot (or all? Or some?) are vertically aligned then don't split them
			}
		}
		ROS_INFO_STREAM("Eroded region "<<timeout<<" times");
		//cull small regions
		for(int blob_index=0;blob_index<temp_regions.size();blob_index++) {
			if(temp_regions[blob_index].size()>300) {
				new_blobs->push_back(temp_regions[blob_index]);
			}
		}
	}
	/*
	 * Name: CombineRegions
	 * Description: combines regions if they are close to eachother
	 */
	void CombineRegions(cv::Mat& region, RegionList* new_blobs) {
		RegionList temp_blobs;
		int timeout=0;

		//dilate the region a maximum of kMaxRegionSplitTimes
		do {
			temp_blobs.clear();
			cv::Mat temp_region;
			//dilate the region and re-find blobs in hopes of getting fewer blobs
			cv::dilate(region,region,cv::Mat());
			region.copyTo(temp_region);
			cv::findContours(temp_region,temp_blobs,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			timeout++;
		} while(temp_blobs.size()>1 && timeout <kMaxRegionCombineTimes);

		ROS_INFO_STREAM("Dilated region "<<timeout<<" times");
		ROS_INFO_STREAM("There are "<<temp_blobs.size()<<" separate regions left");
		//add all regions to the new_blobs
		for(int blob_index=0;blob_index<temp_blobs.size();blob_index++) {
			new_blobs->push_back(temp_blobs[blob_index]);
		}
	}

	void ComputeHistogram(const cv::Mat& image, const cv::Mat& mask, cv::MatND* histogram) {

		 /// Using 30 bins for hue and 32 for saturation
		  int h_bins = 30; int s_bins = 32;
		  int hist_size[] = { h_bins, s_bins };

		  // hue varies from 0 to 256, saturation from 0 to 180
		  float h_ranges[] = { 0, 180 };
		  float s_ranges[] = { 0, 255 };

		  const float* ranges[] = { h_ranges, s_ranges };

		  // Use the o-th and 1-st channels
		  int channels[] = { 0, 1 };

		//compute
		cv::calcHist(&image,1,channels,mask,*histogram,2,hist_size,ranges,true,false);
		cv::normalize(*histogram,*histogram,0,0.999,cv::NORM_MINMAX,-1,cv::Mat());
	}

	cv::Mat MakeBinaryMask(const cv::Mat& image) {
		cv::Mat out;
	    cv::cvtColor(image, out, CV_BGR2GRAY);
	    cv::threshold(out,out,0,255,cv::THRESH_BINARY);
	    return out;
	}

	bool AreVerticallyAligned(const Region& first, const Region& second) {

		if(first.size()<2 || second.size()<2) return false;
		cv::Rect first_bounding_rect = cv::boundingRect(first);
		cv::Rect second_bounding_rect = cv::boundingRect(second);

		int first_x_center = first_bounding_rect.x+first_bounding_rect.width/2;
		int second_x_center = second_bounding_rect.x+second_bounding_rect.width/2;
		int average_width = (first_bounding_rect.width+second_bounding_rect.width)/2;
		//if the regions are vertically separated by less than 1/2 their average widths
		if(abs(first_x_center-second_x_center)<average_width/2) {
			//if they are vertically separated by less than 30 (that's an arbitrary number) pixels
			int top_lower_y;
			int bottom_upper_y;
			if(first_bounding_rect.y<second_bounding_rect.y) {
				top_lower_y = first_bounding_rect.y+first_bounding_rect.height;
				bottom_upper_y = second_bounding_rect.y;
			}
			else {
				top_lower_y = second_bounding_rect.y+second_bounding_rect.height;
				bottom_upper_y = first_bounding_rect.y;
			}
			if(abs(bottom_upper_y-top_lower_y < 30)) {
				//check if heights and widths are compatible
					//the second height differs by no more than 50% of each height
					int height_difference = abs(first_bounding_rect.height-second_bounding_rect.height);
					//height_difference*=1.;
					if(height_difference<first_bounding_rect.height && height_difference<second_bounding_rect.height) {
						return true;
					}
			}
		}
		return false;
	}
	bool AreVerticallyAlignedOriented(const Region& first, const Region& second) {
		if(first.size()<2 || second.size()<2) return false;
		std::cout<<"first area: "<<cv::contourArea(first)<<" second: "<<cv::contourArea(second)<<std::endl;
		cv::Rect upper_bounding_rect = cv::boundingRect(first);
		cv::Rect lower_bounding_rect = cv::boundingRect(second);

		int upper_x_center = upper_bounding_rect.x+upper_bounding_rect.width/2;
		int lower_x_center = lower_bounding_rect.x+lower_bounding_rect.width/2;
		int average_width = (upper_bounding_rect.width+lower_bounding_rect.width)/2;
		//if the regions are vertically separated by less than 1/2 their average widths
		if(abs(upper_x_center-lower_x_center)<average_width/2) {
			//if they are vertically separated by less than 30 (that's an arbitrary number) pixels
			int top_lower_y;
			int bottom_upper_y;
				top_lower_y = upper_bounding_rect.y+upper_bounding_rect.height;
				bottom_upper_y = lower_bounding_rect.y;
			//enforce the top-over-bottom constraint
				if(abs(bottom_upper_y-top_lower_y) < 30) {
					//check if heights and widths are compatible
						//the second height differs by no more than 50% of each height
						int height_difference = abs(upper_bounding_rect.height-lower_bounding_rect.height);
						//height_difference*=1.;
						if(height_difference<upper_bounding_rect.height && height_difference<lower_bounding_rect.height) {
							return true;
						}
				}
		}
		return false;
	}
	bool IsWide(const Region& region) {
		cv::Rect bounding_box = cv::boundingRect(region);
		return (bounding_box.height/bounding_box.width) < 2;
	}

	bool RectsOverlap(const cv::Rect& rhs, const cv::Rect& lhs) {
		cv::Point rhs_center = utility::calcCenter(rhs);
		cv::Point lhs_center = utility::calcCenter(lhs);

		bool rhs_contains_lhs = rhs.contains(lhs_center);
		bool lhs_contains_rhs = lhs.contains(rhs_center);

		return rhs_contains_lhs || lhs_contains_rhs;
	}
	void GetCandidates(const RegionList& regions,const cv::Rect& known_location, std::vector<int>* candidates) {
	for(uint region_index=0;region_index<regions.size();region_index++) {
			if(cv::contourArea(regions[region_index])>kMinRegionSize) {
				cv::Rect this_region_bounding_rect = cv::boundingRect(regions[region_index]);
					candidates->push_back(region_index);
			}
		}
	}
	void GetCandidatesFull(const RegionList& regions,const cv::Rect& known_location, std::vector<int>* candidates) {
	for(uint region_index=0;region_index<regions.size();region_index++) {
			if(cv::contourArea(regions[region_index])>kMinRegionSize) {
				cv::Rect this_region_bounding_rect = cv::boundingRect(regions[region_index]);
				if(utility::RectsOverlap(this_region_bounding_rect,known_location)) {
					candidates->push_back(region_index);
				}
			}
		}
	}

	int CheckForPersonUsingColorScheme(const PersonList& known_blobs,cv::Scalar upper_color,cv::Scalar lower_color) {
		for(unsigned int blob_index=0;blob_index<known_blobs.size();blob_index++) {
			if(known_blobs[blob_index].color[R_UPPER_COLOR] == upper_color &&
			   known_blobs[blob_index].color[R_LOWER_COLOR] == lower_color) {
				return blob_index;
			}
		}
		return -1;
	}
	int CheckForBagUsingColorScheme(const BagList& known_bags,cv::Scalar color) {
		for(unsigned int bag_index=0;bag_index<known_bags.size();bag_index++) {
			if(known_bags[bag_index].color == color) {
				return bag_index;
			}
		}
		return -1;
	}

	void DeleteRegion(int region_index,RegionList* regions) {
		regions->erase(regions->begin()+region_index);
	}

	cv::Rect enlarge(const cv::Rect& input,int amount) {
		return cv::Rect(input.x-amount,input.y-amount,input.width+2*amount,input.height+2*amount);
	}

	bool CheckForHeavySkew(const Region& region) {
		cv::Moments region_moments;
		region_moments = cv::moments(region,false);
		cv::Point2f region_hu_center( region_moments.m10/region_moments.m00 , region_moments.m01/region_moments.m00 );
		cv::Rect region_bounding_rect = cv::boundingRect(region);
		int distance_to_right_side,distance_to_left_side;
		distance_to_right_side = (region_bounding_rect.x+region_bounding_rect.width) - region_hu_center.x;
		distance_to_left_side = region_hu_center.x-region_bounding_rect.x;
		//if the distances vary by more than 25% then trim them
		int tolerance = (distance_to_right_side < distance_to_left_side) ? distance_to_right_side : distance_to_left_side;
		return (abs(distance_to_left_side-distance_to_right_side) > tolerance);
	}

	void RemoveHeavySkew(const cv::Size& frame_size,const RegionList& old_regions,int old_region_index,Region* new_region) {
		cv::Moments region_moments;
		region_moments = cv::moments(old_regions[old_region_index],false);
		cv::Point2f region_hu_center( region_moments.m10/region_moments.m00 , region_moments.m01/region_moments.m00 );
		cv::Rect modified_bounding_rect;
		int distance_to_right_side,distance_to_left_side;
		cv::Rect region_bounding_rect = cv::boundingRect(old_regions[old_region_index]);
		distance_to_right_side = (region_bounding_rect.x+region_bounding_rect.width) - region_hu_center.x;
		distance_to_left_side = region_hu_center.x-region_bounding_rect.x;
		//if the distances vary by more than 25% then trim them
		int tolerance = (distance_to_right_side < distance_to_left_side) ? distance_to_right_side : distance_to_left_side;

		modified_bounding_rect.x = region_hu_center.x-tolerance; // the center minus the shorter distance to Hu center
		modified_bounding_rect.width = tolerance*2; //the width is twice the shorter distance to hu center
		modified_bounding_rect.y = region_bounding_rect.y; //just copy y and height from the original bounding rect
		modified_bounding_rect.height = region_bounding_rect.height;
		//TODO: modify the above algorithm to also catch vertical displacements
		//this is hard because we aren't as semetrical vertically as we are horizontally

		//make a mask of the modified rect
		cv::Mat mask(frame_size,CV_8UC1,cv::Scalar(0));
		cv::rectangle(mask,modified_bounding_rect,cv::Scalar(255),-1,8);
		cv::Mat temp_region(frame_size,CV_8UC1,cv::Scalar(0));
		cv::drawContours(temp_region,old_regions,old_region_index,cv::Scalar(255),-1,8);
		cv::Mat new_region_mat(frame_size,CV_8UC1,cv::Scalar(0));
		temp_region.copyTo(new_region_mat,mask);
		//refind the modified contour
		RegionList new_region_list;
		cv::findContours(new_region_mat,new_region_list,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		while(new_region_list.size()>1) {
			int min=99999999,min_at=0;
			for(int new_region_index=0;new_region_index<new_region_list.size();new_region_index++) {
				if(cv::contourArea(new_region_list[new_region_index]) < min) {
					min = cv::contourArea(new_region_list[new_region_index]);
					min_at = new_region_index;
				}
			}
			new_region_list.erase(new_region_list.begin()+min_at);
		}
		(*new_region) = new_region_list[0];
	}

	void GetSignificantRegions(const cv::Size& frame_size, const std::vector<cv::Rect>& candidate_areas,const cv::Mat& projection,RegionList* regions) {
		regions->clear();
		RegionList all_regions;
		cv::findContours(projection,all_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

		//eliminate small regions
			//this is actually done by copying them to a new array under the
			//assumption that the number of large regions will be small, so it
			//is faster to copy the few then move all of them while deleting
			//them

		for(uint region_index=0;region_index<all_regions.size();region_index++) {
			int area = cv::contourArea(all_regions[region_index]);
			if(area > kMinRegionSize) {
				if(area < kMaxRegionSize) {
					//calculate the hu moment (mass center) and trim the edges if necessary
					if(CheckForHeavySkew(all_regions[region_index])) {
						Region new_region;
						RemoveHeavySkew(frame_size, all_regions,region_index,&new_region);
						regions->push_back(new_region);
					}
					else {
						regions->push_back(all_regions[region_index]);
					}

				}
				else { //try to shrink the large region
					//draw a mask of the significant regions
					//show the huge region
					cv::Mat huge_region(cv::Size(640,480),CV_8UC1,cv::Scalar(0));
					cv::drawContours(huge_region,all_regions,region_index,cv::Scalar(255),-1,8);
					if(candidate_areas.size()>0) {
						int old_size = all_regions.size();
						cv::Mat mask(frame_size,CV_8UC1,cv::Scalar(0));
						for(uint candidate_area_index=0;candidate_area_index<candidate_areas.size();candidate_area_index++) {
							//make the rectangle a little bigger
							cv::rectangle(mask,enlarge(candidate_areas[candidate_area_index],10),cv::Scalar(255),-1,8);
						}
						//draw the large contour
						cv::Mat the_contour(frame_size,CV_8UC1,cv::Scalar(0));
						cv::drawContours(the_contour,all_regions,region_index,cv::Scalar(255),-1,8);
						cv::Mat new_regions_mat;
						the_contour.copyTo(new_regions_mat,mask);
						//find regions again
						RegionList new_regions;
						cv::findContours(new_regions_mat,new_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
						//push the new regions into all_regions
						for(uint new_region_index=0;new_region_index<new_regions.size();new_region_index++) {
							regions->push_back(new_regions[new_region_index]);
						}
					}
				}
			}
		}


	}

	cv::Rect CalcOrientedBoundingRect(const cv::Rect& upper, const cv::Rect& lower) {
		cv::Rect out;
		out.x = (upper.x < lower.x) ? upper.x : lower.x;
		out.y = upper.y;
		out.width = ((upper.x+upper.width > lower.x+lower.width) ? upper.x+upper.width : lower.x+lower.width) - out.x;
		out.height = lower.y+lower.height-upper.y;
		return out;
	}

	void NewPerson(const cv::Mat& depth_raw, const cv::Rect& upper, const cv::Rect& lower,const utility::Pair_<cv::Scalar> color, int frame_number, PersonDescriptor* person) {
//		person->depth_position = GetMeanDepth(depth_raw,upper,lower);
		person->last_upper_location = upper;
		person->last_lower_location = lower;
		person->last_location = CalcOrientedBoundingRect(person->last_upper_location,person->last_lower_location);
		person->first_seen = person->last_seen = frame_number;
		person->color = color;
		person->adjusted_color = color;
		person->init_kalman();
	}
	void NewBag(const cv::Mat& depth_raw, const cv::Rect& region, const cv::Scalar& color, int frame_number, BagDescriptor* bag) {
//		bag->depth_position = GetMeanDepth(depth_raw,region);
		bag->last_location = region;
		bag->first_seen = bag->last_seen = frame_number;
		bag->color = color;
		bag->adjusted_color = color;
		bag->init_kalman();
	}

	void GetSpecificColorMap(const cv::Mat& hsv_raw,const cv::Scalar& hsv_sample,cv::Mat* out) {
		int target_hue = hsv_sample[0];
		int target_saturation = hsv_sample[1];
		int target_value = hsv_sample[2];

			std::vector<cv::Mat> planes;
			cv::split(hsv_raw,planes);
			//add one to the hue plane because the value 0 is red, which we care about
			cv::Mat the_ones(hsv_raw.size(),CV_8UC1,cv::Scalar(1));
			planes[0] = planes[0] + the_ones;

			//if the color is red, also include the wrapped around hue
			cv::Mat red_wraparound_fix(hsv_raw.size(),CV_8UC1,cv::Scalar(0));
			if(target_hue < R_RED_WRAP_WINDOW) { // if it is close to 0 degrees
				//threshold the upper region
				planes[0].copyTo(red_wraparound_fix);
				cv::threshold(red_wraparound_fix,red_wraparound_fix,180-R_RED_WRAP_WINDOW,255,cv::THRESH_BINARY);
			}
			else if(target_hue > (180-R_RED_WRAP_WINDOW)) { //if it is close to 360 degrees
				//threshold the lower region
				planes[0].copyTo(red_wraparound_fix);
				cv::threshold(red_wraparound_fix,red_wraparound_fix,R_RED_WRAP_WINDOW,255,cv::THRESH_BINARY_INV);
			}
			//threshold both sides
			int hue_window=8;
			if(target_hue>hue_window) cv::threshold(planes[0],planes[0],target_hue-hue_window,0,cv::THRESH_TOZERO);
			if(target_hue<(180-hue_window)) cv::threshold(planes[0],planes[0],target_hue+hue_window,255,cv::THRESH_TOZERO_INV);
			cv::threshold(planes[0],planes[0],0,255,CV_THRESH_BINARY);
			//or this with the wraparound fix to get both kinds of red
			planes[0] = planes[0] | red_wraparound_fix;
//			imshow("hue slice",planes[0]);

			for(int i=0;i<2;i++) cv::dilate(planes[0],planes[0],cv::Mat());
			for(int i=0;i<2;i++) cv::erode(planes[0],planes[0],cv::Mat());

			//bracket saturation
			int saturation_window=25;
			cv::threshold(planes[1],planes[1],target_saturation-saturation_window,0,cv::THRESH_TOZERO);
			cv::threshold(planes[1],planes[1],target_saturation+saturation_window,255,cv::THRESH_TOZERO_INV);
//			imshow("sat slice",planes[1]);
			for(int i=0;i<2;i++) cv::dilate(planes[1],planes[1],cv::Mat());
			for(int i=0;i<2;i++) cv::erode(planes[1],planes[1],cv::Mat());

			//mask the hue plane with the plausible saturation plane
			cv::Mat black(hsv_raw.size(),CV_8UC1,cv::Scalar(0));
			planes[0].copyTo(*out,planes[1]);
			for(int i=0;i<1;i++) cv::erode(*out,*out,cv::Mat());
	}
	void MaskWithDepth(const cv::Mat& depth_raw, int target_depth,const cv::Mat& in,cv::Mat* out) {
			int depth_window=10;
			cv::threshold(depth_raw,*out,target_depth-depth_window,0,cv::THRESH_TOZERO);
			cv::threshold(*out,*out,target_depth+depth_window,255,cv::THRESH_TOZERO_INV);

			//mask the input
			*out = in & *out;
//			imshow("masked with depth",*out);
	}
	cv::Rect UpperHalf(const cv::Rect in) {
		return cv::Rect(in.x,in.y,in.width,in.height/2);
	}
	cv::Rect LowerHalf(const cv::Rect in) {
		return cv::Rect(in.x,in.y+in.height/2,in.width,in.height/2);
	}
	cv::Rect UpperPersonHalf(const cv::Rect in) {
		return cv::Rect(in.x,in.y,in.width,in.height*0.4);
	}
	cv::Rect LowerPersonHalf(const cv::Rect in) {
		return cv::Rect(in.x,in.y+in.height*0.4,in.width,in.height*0.6);
	}
	void CullList(int frame_number, PersonList* known_people) {
		for(unsigned int person_index = 0;person_index < (*known_people).size();person_index++) {
			bool rarely_seen = (*known_people)[person_index].history.size() <= 2;
			bool not_seen_recently = (frame_number - (*known_people)[person_index].last_seen) >= 3;
			if(rarely_seen && not_seen_recently) {
				(*known_people).erase(known_people->begin()+person_index);
				ROS_INFO_STREAM("Erased the "<<person_index<<" person");
			}
		}
	}
	void CullList(int frame_number, BagList* known_bags) {
		for(unsigned int bag_index = 0;bag_index < (*known_bags).size();bag_index++) {
			bool rarely_seen = (*known_bags)[bag_index].history.size() <= 2;
			bool not_seen_recently = (frame_number - (*known_bags)[bag_index].last_seen) >= 3;
			if(rarely_seen && not_seen_recently) {
				(*known_bags).erase(known_bags->begin()+bag_index);
				ROS_INFO_STREAM("Erased the "<<bag_index<<" bag");
			}
		}
	}

	void UpdatePerson(const cv::Mat& depth_raw, const cv::Scalar& upper_observed_color,const cv::Scalar& lower_observed_color,int frame_number, const cv::Rect& upper,const cv::Rect& lower, PersonDescriptor* person) {
//		person->depth_position = GetMeanDepth(depth_raw,upper,lower);
		person->last_upper_location = upper;
		person->last_lower_location = lower;
		person->last_location = utility::CalcOrientedBoundingRect(person->last_upper_location,person->last_lower_location);
		person->last_seen = frame_number;

//		person->adjusted_color[0] = person->adjusted_color[0]*0.99 + upper_observed_color*0.01;
//		person->adjusted_color[1] = person->adjusted_color[1]*0.99 + lower_observed_color*0.01;


		HistoryDescriptor temp;
		temp.position = person->last_location;
		person->history[frame_number] = temp;
	}

	void UpdateBag(const cv::Mat& depth_raw, const cv::Scalar& observed_color,int frame_number, const cv::Rect& region, BagDescriptor* bag) {
//		bag->depth_position = GetMeanDepth(depth_raw,region);
		bag->last_location = region;
		bag->last_seen = frame_number;

//		bag->adjusted_color = bag->adjusted_color*0.99 + observed_color*0.01;

		HistoryDescriptor temp;
		temp.position = bag->last_location;
		bag->history[frame_number] = temp;
	}

	void UpdateKalmans(int frame_number, PersonList* known_people) {
		for(uint person_index=0;person_index<known_people->size();person_index++) {
			(*known_people)[person_index].update_kalman(frame_number);
		}
	}

	void UpdateKalmans(int frame_number, BagList* known_bags) {
		for(uint bag_index=0;bag_index<known_bags->size();bag_index++) {
			(*known_bags)[bag_index].update_kalman(frame_number);
		}
	}

	cv::Scalar HSV2BGR(cv::Scalar HSV) {
		cv::Scalar BGR;
		cv::Mat temp(cv::Size(1,1),CV_8UC3,HSV);
		cvtColor(temp,temp,CV_HSV2BGR);
		BGR[0] = temp.at<cv::Vec3b>(0)[0];
		BGR[1] = temp.at<cv::Vec3b>(0)[1];
		BGR[2] = temp.at<cv::Vec3b>(0)[2];

		return BGR;
	}

	cv::Scalar BGR2HSV(cv::Scalar BGR) {
		cv::Scalar HSV;
		cv::Mat temp(cv::Size(1,1),CV_8UC3,BGR);
		cvtColor(temp,temp,CV_BGR2HSV);
		HSV[0] = temp.at<cv::Vec3b>(0)[0];
		HSV[1] = temp.at<cv::Vec3b>(0)[1];
		HSV[2] = temp.at<cv::Vec3b>(0)[2];

		return HSV;
	}


	void AddCurrentProjection(const cv::Scalar& hsv_sample,const cv::Mat& hsv_raw, cv::Mat* out) {
		if(!out->data) {
			out->create(hsv_raw.size(),CV_8UC3);
			out->setTo(cv::Scalar(0,0,0));
		}
		//test if it is light or dark
		cv::Mat mask;

		utility::GetSpecificColorMap(hsv_raw,hsv_sample,&mask);

		cv::threshold(mask,mask,1,255,CV_THRESH_BINARY);


		cv::Mat color_frame(hsv_raw.size(),CV_8UC3,HSV2BGR(hsv_sample));
		cv::Mat projection(hsv_raw.size(),CV_8UC3,cv::Scalar(0));

		color_frame.copyTo(projection,mask);

		*out = *out | projection;

	}

	bool AreClose(const cv::Rect& lhs, const cv::Rect& rhs) {
		cv::Point lhs_center(lhs.x+lhs.width/2,lhs.y+lhs.height/2);
		cv::Point rhs_center(rhs.x+rhs.width/2,rhs.y+rhs.height/2);
		int distance = pow(lhs_center.x-rhs_center.x,2)+pow(lhs_center.y-rhs_center.y,2);
		distance = sqrt(distance);
		if(distance < (lhs.width+rhs.width)) return true;
		else return false;
	}

	bool AreCloseX(const cv::Rect& lhs, const cv::Rect& rhs) {
		int lhs_x = lhs.x+lhs.width/2;
		int rhs_x = rhs.x+rhs.width/2;
		int distance = abs(lhs_x-rhs_x);
		if(distance < (lhs.width+rhs.width)) return true;
		else return false;
	}

	bool AreClose3d(const cv::Rect& lhs, const cv::Rect& rhs,int lhs_depth,int rhs_depth) {
		if(abs(lhs_depth-rhs_depth) > kDepthClosenessThreshold) return false;

		cv::Point lhs_center(lhs.x+lhs.width/2,lhs.y+lhs.height/2);
		cv::Point rhs_center(rhs.x+rhs.width/2,rhs.y+rhs.height/2);
		int distance = pow(lhs_center.x-rhs_center.x,2)+pow(lhs_center.y-rhs_center.y,2);
		distance = sqrt(distance);
		distance = distance - distance*0.5;
		if(distance < (lhs.width+rhs.width)) return true;
		else return false;
	}

	int GetMeanDepth(const cv::Mat& depth_raw,const Region& upper,const Region& lower) {
		//make a mask of the regions
		cv::Mat mask(depth_raw.size(),CV_8UC1,cv::Scalar(0));
		RegionList both_regions;
		both_regions.push_back(upper);
		if(lower.size()>0) both_regions.push_back(lower);
		cv::drawContours(mask,both_regions,-1,cv::Scalar(255),-1,8);

		//find mean from the depth mat
		return cv::mean(depth_raw,mask)[0];
	}
	int GetMeanDepth(const cv::Mat& depth_raw,const Region& region) {
		return GetMeanDepth(depth_raw, region,Region());
	}

	bool CheckForInheldRect(const cv::Rect& rhs, const cv::Rect& lhs) {
		cv::Point rhs_center = utility::calcCenter(rhs);
		cv::Point lhs_center = utility::calcCenter(lhs);

		bool rhs_contains_lhs = rhs.contains(lhs_center);
		bool lhs_contains_rhs = lhs.contains(rhs_center);

		return rhs_contains_lhs || lhs_contains_rhs;
	}

	bool Normalize(int frame_number, const cv::Mat& hsv_raw,cv::Mat* color, cv::Scalar* last_mean) {
		cv::Scalar mean = cv::mean(hsv_raw);
		if(frame_number>1 && abs(mean[2]-(*last_mean)[2]) > 3) {
//			ROS_INFO_STREAM("Camera brightness adjustment detected, attempting to compensate.");
			//get weighted normalization factor
			double factor;
			factor = (mean[2]*0.25+(*last_mean)[2]*0.75)/mean[2];
			*color = *color * factor;
			*last_mean = mean;
			(*last_mean)[2] *=factor;
			return true;
		}
		else {
			(*last_mean) = mean;
			return false;
		}
	}

	void MaskToZero(const cv::Mat& mask, cv::Mat* input_output) {
		cv::Mat temp(input_output->size(),input_output->type(),cv::Scalar(0));
		input_output->copyTo(temp,mask);
		temp.copyTo(*input_output);
	}

}
#endif
