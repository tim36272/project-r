#ifndef TIM_SRC_UTILITIES_CPP
#define TIM_SRC_UTILITIES_CPP

//histogram calculation
#define R_UPPER_COLOR 0
#define R_LOWER_COLOR 1
#define R_RED_WRAP_WINDOW 15

#include <ros/console.h>
#include <iostream>
#include "BlobDescriptor.h"
#include "Types.h"

//region splitting constants
static const int kMaxRegionSplitTimes = 10;
static const int kMaxRegionCombineTimes = 5;

static const int kMinRegionSize=500;
static const int kMaxRegionSize=10000;

typedef std::vector<cv::Point> Region;
typedef std::vector<Region> RegionList;
typedef std::vector<PersonDescriptor> PersonList;
typedef std::vector<BagDescriptor> BagList;
typedef const cv::Mat& kMat;

namespace utility {
	//protoypes
	cv::Point calcCenter(const cv::Rect& rectangle);
	void SplitRegion(const cv::Point& mass_center, cv::Mat& region, RegionList* new_blobs);
	void CombineRegions(cv::Mat& region, RegionList* new_blobs);
	void ComputeHistogram(const cv::Mat& image, const cv::Mat& mask, cv::MatND* histogram);
	cv::Mat MakeBinaryMask(const cv::Mat& image);
	bool AreVerticallyAligned(const Region& first, const Region& second);
	bool IsWide(const Region& region);
	bool RectsOverlap(const cv::Rect& rhs, const cv::Rect& lhs);
	void GetCandidates(const RegionList& regions,const cv::Rect& known_location, std::vector<int>* candidates);
	void GetCandidatesFull(const RegionList& regions,const cv::Rect& known_location, std::vector<int>* candidates);
	void CheckForPersonUsingColorScheme(const PersonList& known_blobs,cv::Scalar upper_color,cv::Scalar lower_color,std::vector<int>* blobs_using);
	void CheckForBagUsingColorScheme(const BagList& known_bags,cv::Scalar color,std::vector<int>* bags_using);
	void DeleteRegion(int region_index,RegionList* regions);
	void GetSignificantRegions(const cv::Size& frame_size, const std::vector<cv::Rect>& candidate_areas,const cv::Mat& projection,RegionList* regions);
	cv::Rect CalcOrientedBoundingRect(const cv::Rect& upper, const cv::Rect& lower);
	void NewPerson(const Region& upper, const Region& lower,const utility::Pair_<cv::Scalar> color, int frame_number, PersonDescriptor* person);
	void NewBag(const Region& region,const cv::Scalar color, int frame_number, int owner, BagDescriptor* bag);
	void GetSpecificColorMap(const cv::Mat& hsv_raw,const cv::Scalar& color,cv::Mat* out,bool dark);
	cv::Rect UpperHalf(const cv::Rect in);
	cv::Rect LowerHalf(const cv::Rect in);
	void CullList(int frame_number, PersonList* known_people);
	void CullList(int frame_number, BagList* known_bags);
	void UpdatePerson(int frame_number, const Region& upper,const Region& lower, PersonDescriptor* person);
	void UpdateBag(int frame_number, const Region& region, BagDescriptor* bag);
	Pair TryToUpdatePerson(int frame_number, const RegionList& upper_regions, const RegionList& lower_regions, PersonDescriptor* person);
	int TryToUpdateBag(int frame_number, const RegionList& regions, const cv::Rect owner_location, BagDescriptor* bag);
	void UpdateKalmans(int frame_number, PersonList* known_people);
	void UpdateKalmans(int frame_number, BagList* known_bags);
	void AddCurrentProjection(const cv::Scalar& color,const cv::Mat& hsv_raw, cv::Mat* out);


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
			if(bottom_upper_y-top_lower_y < 30) {
				return true;
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

	void CheckForPersonUsingColorScheme(const PersonList& known_blobs,cv::Scalar upper_color,cv::Scalar lower_color,std::vector<int>* blobs_using) {
		for(unsigned int blob_index=0;blob_index<known_blobs.size();blob_index++) {
			if(known_blobs[blob_index].color[R_UPPER_COLOR] == upper_color &&
			   known_blobs[blob_index].color[R_LOWER_COLOR] == lower_color) {
				blobs_using->push_back(blob_index);
			}
		}
	}
	void CheckForBagUsingColorScheme(const BagList& known_bags,cv::Scalar color,std::vector<int>* bags_using) {
		for(unsigned int bag_index=0;bag_index<known_bags.size();bag_index++) {
			if(known_bags[bag_index].color == color) {
				bags_using->push_back(bag_index);
			}
		}
	}

	void DeleteRegion(int region_index,RegionList* regions) {
		regions->erase(regions->begin()+region_index);
	}

	cv::Rect enlarge(const cv::Rect& input,int amount) {
		return cv::Rect(input.x-amount,input.y-amount,input.width+2*amount,input.height+2*amount);
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
					cv::Moments region_moments;
					region_moments = cv::moments(all_regions[region_index],false);
					cv::Point2f region_hu_center( region_moments.m10/region_moments.m00 , region_moments.m01/region_moments.m00 );
					cv::Rect region_bounding_rect = cv::boundingRect(all_regions[region_index]);
					int distance_to_right_side,distance_to_left_side;
					distance_to_right_side = (region_bounding_rect.x+region_bounding_rect.width) - region_hu_center.x;
					distance_to_left_side = region_hu_center.x-region_bounding_rect.x;
					//if the distances vary by more than 25% then trim them
					int tolerance = (distance_to_right_side < distance_to_left_side) ? distance_to_right_side : distance_to_left_side;
					if(abs(distance_to_left_side-distance_to_right_side) > tolerance) {
						std::cout<<"This region is heavily skewed"<<std::endl;
						cv::Rect modified_bounding_rect;
						modified_bounding_rect.x = region_hu_center.x-tolerance; // the center minus the shorter distance to Hu center
						modified_bounding_rect.width = tolerance*2; //the width is twice the shorter distance to hu center
						modified_bounding_rect.y = region_bounding_rect.y; //just copy y and height from the original bounding rect
						modified_bounding_rect.height = region_bounding_rect.height;
						std::cout<<"modified rect: "<<modified_bounding_rect<<std::endl;
						//TODO: modify the above algorithm to also catch vertical displacements
						//this is hard because we aren't as semetrical vertically as we are horizontally

						//make a mask of the modified rect
						cv::Mat mask(frame_size,CV_8UC1,cv::Scalar(0));
						cv::rectangle(mask,modified_bounding_rect,cv::Scalar(255),-1,8);
						cv::Mat temp_region(frame_size,CV_8UC1,cv::Scalar(0));
						cv::drawContours(temp_region,all_regions,region_index,cv::Scalar(255),-1,8);
						cv::Mat new_region_mat(frame_size,CV_8UC1,cv::Scalar(0));
						temp_region.copyTo(new_region_mat,mask);
						//refind the modified contour
						RegionList new_region;
						cv::findContours(new_region_mat,new_region,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
						while(new_region.size()>1) {
							int min=99999999,min_at=0;
							for(int new_region_index=0;new_region_index<new_region.size();new_region_index++) {
								if(cv::contourArea(new_region[new_region_index]) < min) {
									min = cv::contourArea(new_region[new_region_index]);
									min_at = new_region_index;
								}
							}
							new_region.erase(new_region.begin()+min_at);
						}
						regions->push_back(new_region[0]);
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
					imshow("Huge region",huge_region);
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
						imshow("new regions from large region",new_regions_mat);
						//find regions again
						RegionList new_regions;
						cv::findContours(new_regions_mat,new_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
						//push the new regions into all_regions
						for(uint new_region_index=0;new_region_index<new_regions.size();new_region_index++) {
							all_regions.push_back(new_regions[new_region_index]);
						}
						//erase the large region
						all_regions.erase(all_regions.begin()+region_index);
						//modify the region_index since we changed the vector
						if(region_index<(old_size-1)) region_index--;
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

	void NewPerson(const Region& upper, const Region& lower,const utility::Pair_<cv::Scalar> color, int frame_number, PersonDescriptor* person) {
		person->last_upper_location = cv::boundingRect(upper);
		person->last_lower_location = cv::boundingRect(lower);
		person->last_location = CalcOrientedBoundingRect(person->last_upper_location,person->last_lower_location);
		person->first_seen = person->last_seen = frame_number;
		person->color = color;
		person->init_kalman();
	}
	void NewBag(const Region& region,const cv::Scalar color, int frame_number, int owner, BagDescriptor* bag) {
		bag->last_location = boundingRect(region);
		bag->first_seen = bag->last_seen = frame_number;
		bag->color = color;
		bag->belongs_to = owner;
		bag->init_kalman();
	}

	void GetSpecificColorMap(const cv::Mat& hsv_raw,const cv::Scalar& hsv_sample,cv::Mat* out,bool dark=false) {
		int target_hue = hsv_sample[0];
		int target_saturation = hsv_sample[1];
		int target_value = hsv_sample[2];

			std::vector<cv::Mat> planes;
			cv::split(hsv_raw,planes);
			//add one to the hue plane
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
			int hue_window=/*10*/40;
			if(target_hue>hue_window) cv::threshold(planes[0],planes[0],target_hue-hue_window,0,cv::THRESH_TOZERO);
			if(target_hue<(180-hue_window)) cv::threshold(planes[0],planes[0],target_hue+hue_window,255,cv::THRESH_TOZERO_INV);
			cv::threshold(planes[0],planes[0],0,255,CV_THRESH_BINARY);
			//or this with the wraparound fix
			planes[0] = planes[0] | red_wraparound_fix;
			for(int i=0;i<2;i++) cv::dilate(planes[0],planes[0],cv::Mat());
			for(int i=0;i<3;i++) cv::erode(planes[0],planes[0],cv::Mat());

			//bracket saturation
			int saturation_window=35;
			cv::threshold(planes[1],planes[1],target_saturation-saturation_window,0,cv::THRESH_TOZERO);
			cv::threshold(planes[1],planes[1],target_saturation+saturation_window,255,cv::THRESH_TOZERO_INV);
			for(int i=0;i<2;i++) cv::dilate(planes[1],planes[1],cv::Mat());
			for(int i=0;i<2;i++) cv::erode(planes[1],planes[1],cv::Mat());

			//mask the hue plane with the plausible saturation plane
			cv::Mat black(hsv_raw.size(),CV_8UC1,cv::Scalar(0));
			planes[0].copyTo(black,planes[1]);
			black.copyTo(*out/*,planes[2]*/);
			for(int i=0;i<1;i++) cv::erode(*out,*out,cv::Mat());
	}
	cv::Rect UpperHalf(const cv::Rect in) {
		return cv::Rect(in.x,in.y,in.width,in.height/2);
	}
	cv::Rect LowerHalf(const cv::Rect in) {
		return cv::Rect(in.x,in.y+in.height/2,in.width,in.height/2);
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

	void UpdatePerson(int frame_number, const Region& upper,const Region& lower, PersonDescriptor* person) {
		person->last_upper_location = cv::boundingRect(upper);
		person->last_lower_location = cv::boundingRect(lower);
		person->last_location = utility::CalcOrientedBoundingRect(person->last_upper_location,person->last_lower_location);
		person->last_seen = frame_number;

		HistoryDescriptor temp;
		temp.position = person->last_location;
		person->history[frame_number] = temp;
	}

	void UpdateBag(int frame_number, const Region& region, BagDescriptor* bag) {
		bag->last_location = boundingRect(region);
		bag->last_seen = frame_number;

		HistoryDescriptor temp;
		temp.position = bag->last_location;
		bag->history[frame_number] = temp;
	}

	Pair TryToUpdatePerson(int frame_number, const RegionList& upper_regions, const RegionList& lower_regions, PersonDescriptor* person) {
		//find regions vertically aligned in the search window
		std::vector<int> upper_candidates,lower_candidates;
		utility::GetCandidates(upper_regions,utility::UpperHalf(person->filter.bounding_rect()),&upper_candidates);
		utility::GetCandidates(lower_regions,utility::LowerHalf(person->filter.bounding_rect()),&lower_candidates);
		//show candidates
		cv::Mat candidates_1(cv::Size(640,480),CV_8UC3,cv::Scalar(0,0,0));
		for(uint upper_index=0;upper_index<upper_candidates.size();upper_index++) {
			cv::drawContours(candidates_1,upper_regions,upper_candidates[upper_index],cv::Scalar(255,0,0),-1,8);
		}
		for(uint lower_index=0;lower_index<lower_candidates.size();lower_index++) {
			cv::drawContours(candidates_1,lower_regions,lower_candidates[lower_index],cv::Scalar(0,255,0),-1,8);
		}
		imshow("original candidates",candidates_1);

		std::vector<utility::Pair> full_candidates;

		//search each pair of regions to see if they are vertically aligned
		for(uint upper_candidate_index=0; upper_candidate_index<upper_candidates.size(); upper_candidate_index++) {
			for(uint lower_candidate_index=0; lower_candidate_index<lower_candidates.size(); lower_candidate_index++) {
				if(utility::AreVerticallyAligned(upper_regions[upper_candidate_index],lower_regions[lower_candidate_index])) {
					full_candidates.push_back(utility::Pair(upper_candidate_index,lower_candidate_index));
				}
			}
		}
		if(full_candidates.size()==1) {
			UpdatePerson(frame_number, upper_regions[full_candidates[0].values[0]],lower_regions[full_candidates[0].values[1]],person);
			return full_candidates[0];
		}
		else if(full_candidates.size()>1) {
			cv::Mat full_candidates_mat(cv::Size(640,480),CV_8UC3,cv::Scalar(0,0,0));
			for(int i=0;i<full_candidates.size();i++) {
				cv::Scalar color(rand()%255,rand()%255,rand()%255);
				cv::drawContours(full_candidates_mat,upper_regions,full_candidates[i][0],color,-1,8);
				cv::drawContours(full_candidates_mat,lower_regions,full_candidates[i][1],color,-1,8);
			}
			imshow("Full candidates",full_candidates_mat);
			ROS_ERROR_STREAM("cannot arbitrate "<<full_candidates.size()<<" full candidates");
		}
		return utility::Pair();
	}

	int TryToUpdateBag(int frame_number, const RegionList& regions, const cv::Rect owner_location, BagDescriptor* bag) {
		//find regions vertically aligned in the search window
		std::vector<int> candidates;

		//first look where the bag should be
		GetCandidatesFull(regions,bag->filter.bounding_rect(),&candidates);

		//if that list is empty then look around the person
		if(candidates.size()==0) {
			//make a rect three times the width of the person
			cv::Rect search_window(owner_location.x-owner_location.width,
								   owner_location.y+owner_location.height/2,
								   owner_location.width*3,
								   owner_location.height/2);
			GetCandidatesFull(regions,search_window,&candidates);
		}

		if(candidates.size()==1) {
			UpdateBag(frame_number, regions[candidates[0]],bag);
			return candidates[0];
		}
		else if(candidates.size()>1) {
			ROS_ERROR_STREAM("cannot arbitrate "<<candidates.size()<<" full candidates");
		}
		return -1;
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

}
#endif
