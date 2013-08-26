#include "Analysis.h"

#define HISTOGRAM_COMPARE_METHOD CV_COMP_CHISQR

static const int kMinRegionSize=1500;
static const int kSameLocationTimeWindow=5;


static const double kMinWeakRelation=0.6;

Analysis::Analysis() {
	frame_number_ = 0;
	color_segmentation_.set("nmixtures",3);
	depth_segmentation_.set("nmixtures",3);
	depth_segmentation_.set("detectShadows",0);
}

//takes a new color and depth frame and modifies the known segmentation
int Analysis::AddFrame(const cv::Mat &color_raw, const cv::Mat &depth_raw) {
	//this is used to know when a blob was last seen
	frame_number_++;
	//make a shallow copy
	color_raw_ = color_raw;
	depth_raw_ = depth_raw;

	//update the color segmentation
		//perform the update
		color_segmentation_(color_raw,color_foreground_);
		//threshold shadows out of color image
		cv::threshold(color_foreground_,color_foreground_,129,0,cv::THRESH_TOZERO);
		//smoothing for color image
		cv::dilate(color_foreground_,color_foreground_,cv::Mat());
//		cv::erode(color_foreground_,color_foreground_,cv::Mat());

	//update the depth segmentation
		depth_segmentation_(depth_raw,depth_foreground_);
		//smoothing
//		cv::dilate(color_foreground_,color_foreground_,cv::Mat());
		cv::erode(depth_foreground_,depth_foreground_,cv::Mat());


	//create a unified foreground
		combined_foreground_ = depth_foreground_/* & color_foreground_*/;

	return frame_number_;
}

//passes back a RegionList of interesting points from the segmentation
void Analysis::GetRegions(RegionList* regions) {
	//use combined foreground to mask depth image to src
	cv::Mat depth_masked;
	depth_raw_.copyTo(depth_masked,combined_foreground_);
	RegionList temp_regions;
	double minVal,maxVal;
	//FindDepthMaskContours(depth_masked,&temp_regions);

	//cv::findContours(depth_masked,*regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	cv::findContours(combined_foreground_,*regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
}

void Analysis::CullSmallRegions(RegionList* regions) {
	for(int region_index=0;region_index<regions->size();region_index++) {
		if(cv::contourArea((*regions)[region_index])<kMinRegionSize) {
			regions->erase(regions->begin()+region_index);
		}
	}
}

void Analysis::SplitCollidedRegions(const std::vector<BlobDescriptor>& known_blobs, RegionList* regions) {
	std::vector<std::vector<int> > collision_candidates;
	utility::FindCollisionCandidates(known_blobs,&collision_candidates);

	//get new regions which overlap each collision region
	//the first index in overlapping_regions is the first blob they collide with, the second is a list of regions overlapping those blobs
	std::vector<std::vector<int> > overlapping_regions;
	utility::GetOverlappingRegions(collision_candidates,*regions,known_blobs,&overlapping_regions);

	//draw the overlapping regions on a Mat
	for(int collision_index=0;collision_index<overlapping_regions.size();collision_index++) {
		cv::Mat this_collision_set(color_raw_.size(),CV_8UC1,cv::Scalar(0));
		for(int region_index=0;region_index<overlapping_regions[collision_index].size();region_index++) {
			cv::drawContours(this_collision_set,*regions,overlapping_regions[collision_index][region_index],cv::Scalar(255),-1,8);
		}
		//erase the overlapping area
		for(int second_blob_index=0;second_blob_index<collision_candidates[collision_index].size();second_blob_index++) {
			int other_blob = collision_candidates[collision_index][second_blob_index];
			cv::Rect shared = utility::CalculateSharedRect(known_blobs[collision_index].filter.bounding_rect_estimate(),
														   known_blobs[other_blob].filter.bounding_rect_estimate());
			cv::rectangle(this_collision_set,shared,cv::Scalar(0),-1,8);
		}

/*		if(overlapping_regions[collision_index].size()>1) {
			imshow("colliding_regions",this_collision_set);
		}*/

		//find contours in the new Mat
		RegionList new_regions;
		cv::findContours(this_collision_set,new_regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		cv::Mat test2(color_raw_.size(),CV_8UC1,cv::Scalar(0));
		drawContours(test2,new_regions,-1,cv::Scalar(255),-1,8);
	/*	if(overlapping_regions[collision_index].size()>1) {
			imshow("found overlap",test2);
		}*/

		//replace the regions in overlapping_regions with new_regions
		utility::ReplaceRegions(color_raw_,overlapping_regions[collision_index],new_regions,regions);

		cv::Mat test(color_raw_.size(),CV_8UC1,cv::Scalar(0));
		drawContours(test,*regions,-1,cv::Scalar(255),-1,8);
		if(collision_index==0) {
			imshow("all new contours",test);
			cv::waitKey(100);
		}
	}
}

//make a vector of BlobDescriptors for all the significant regions found
void Analysis::AnalyzeRegions(const RegionList& regions,std::vector<BlobDescriptor>* current_blobs) {
	//compute a histogram for each blob and add it to the current blobs
	cv::Mat all_contours(color_raw_.size(),CV_8UC3,cv::Scalar(0));
	for(unsigned int blob_index=0;blob_index<regions.size();blob_index++) {
		//require a certain size and certain density
		if(cv::contourArea(regions[blob_index]) > kMinRegionSize) {
			std::cout<<"area: "<<cv::contourArea(regions[blob_index])<<std::endl;
			cv::drawContours(all_contours,regions,blob_index,cv::Scalar((blob_index%254)+1,rand()%255,rand()%255),-1,8);
			//make a Mat of the contour
			cv::Point mass_center;
			//if the blob should be split
			if( utility::IsMassCenterOutsideBlob(regions[blob_index],all_contours,(blob_index%254)+1,&mass_center) ) {

				RegionList new_blobs;
				cv::Mat this_contour(color_raw_.size(),CV_8UC1,cv::Scalar(0));
				cv::drawContours(this_contour,regions,blob_index,cv::Scalar(255),-1,8);
				cv::circle(this_contour,mass_center,1,cv::Scalar(128),-1,8);
				utility::SplitRegion(mass_center,this_contour,&new_blobs);

				//add however many blobs we came out with to the chain
				for(int new_blob_index=0;new_blob_index<new_blobs.size();new_blob_index++) {
					//compute the histogram for this blob
					BlobDescriptor this_blob;
					MakeNewBlobDescriptor(new_blobs,new_blob_index,&this_blob);
					//add the blob to the chain
					current_blobs->push_back(this_blob);
					cv::drawContours(all_contours,new_blobs,new_blob_index,cv::Scalar((blob_index%254)+1,rand()%255,rand()%255),-1,8);
				}
			}
			else { //the blob was not split*/
				//draw the contours on the information mat
				//compute the histogram for this blob
				BlobDescriptor this_blob;
				MakeNewBlobDescriptor(regions,blob_index,&this_blob);
				//add the blob to the chain
				current_blobs->push_back(this_blob);
			}
		}
	}
	imshow("blobs",all_contours);
}

//takes a set of histograms and match them to the known set or add them to the set, updating describing information
void Analysis::RegisterBlobs(std::vector<BlobDescriptor>& current_blobs, std::vector<BlobDescriptor>* known_blobs) {

	//for each current blob
	for(unsigned int current_blob_index=0;current_blob_index<current_blobs.size();current_blob_index++) {
		bool blob_updated = false;
		//compare it to each known blobs
		for(unsigned int known_blob_index=0;known_blob_index<known_blobs->size() && !blob_updated;known_blob_index++) {
			//compare the current and known
			blob_updated = RegisterMoving(current_blobs[current_blob_index],&((*known_blobs)[known_blob_index]) );
		}
		//if it wasn't updated elsewhere
		if(!blob_updated) {
			//if it wasn't updated then add the blob to the chain
			known_blobs->push_back(current_blobs[current_blob_index]);
			known_blobs->back().init_kalman();
		}
			//otherwise simply move on to the next blob
	}
}

bool Analysis::RegisterMoving(BlobDescriptor& current_blob, BlobDescriptor* known_blob) {
	bool blob_set = false;
	//if the similarity exceeds a threshold then update it
		//if this blob has already been updated skip it:
		if (current_blob.last_seen == (*known_blob).last_seen) {
			return false;
		}
		//compare the last updated time to now
		assert(current_blob.last_seen > (*known_blob).last_seen);

		if((current_blob.last_seen-(*known_blob).last_seen) < kSameLocationTimeWindow){
			//if it was updated recently then check if it is where the Kalman filter expects it to be
			if(utility::CheckForInheldRect(current_blob.last_location,(*known_blob).filter.bounding_rect_estimate())) {
				//check if this blob is almost completely contained within the known blob
					//find the overlapping area
					int shared_area = utility::CalculateSharedArea(known_blob->filter.bounding_rect_estimate(), current_blob.last_location);
					if( shared_area > current_blob.last_location.area()/2 && current_blob.last_location.area() < known_blob->filter.bounding_rect_estimate().area()/2) {
						//we got a fragment of the blob, so move on with life
						return true;
					}
				//see if the blob is similar enough to be updated
				double real_relation;
				double relation = utility::CompareBlobs((*known_blob), current_blob,HISTOGRAM_COMPARE_METHOD,real_relation);
				if(utility::CheckForStrongRelation(relation,known_blob->relation_mean,known_blob->relation_variance,HISTOGRAM_COMPARE_METHOD)) {
					std::cout<<"Updating blob ID "<<known_blob->first_seen<<" at "<<known_blob->filter.bounding_rect_estimate()<<" with new at "<<current_blob.last_location<<std::endl;
					known_blob->last_relation = real_relation;
					std::cout<<"Real relation: "<<real_relation<<std::endl;
					(*known_blob).update(current_blob);
					blob_set = true;
				}
				else {
					//there must be something occluding this blob because it is way different than what was expected since it was in the same place
					return false;
				}
			}
			else {
				return false; // the blob is in a totally different place
			}
		}
		else {
			//getting here means the blob was possibly occluded a while ago and now re-visible
			//check if it is where the Kalman filter expects it to be
				//first check if it didn't move (if it is at last_location)
				cv::Point new_center = utility::calcCenter(current_blob.last_location);
				bool last_known_location_contains_new_center = (*known_blob).last_location.contains(new_center);

				//then check if it is at the projected position
				cv::Point kalman_center = utility::calcCenter((*known_blob).filter.bounding_rect_estimate());
				bool kalman_prediction_contains_new_center = (*known_blob).filter.bounding_rect_estimate().contains(new_center);
				bool new_blob_contains_kalman_prediction_center = current_blob.last_location.contains(kalman_center);

				if(last_known_location_contains_new_center || kalman_prediction_contains_new_center || new_blob_contains_kalman_prediction_center) {
					double real_relation;
					double relation = utility::CompareBlobs((*known_blob), current_blob,HISTOGRAM_COMPARE_METHOD,real_relation);
					if(relation > kMinWeakRelation) {
						known_blob->last_relation = real_relation;
						(*known_blob).update(current_blob);
						blob_set = true;
					}
					else {
						return false; // the blob is really dissimilar
					}
				}
				else {
					return false; // it is not where it should be
				}
			}
	return blob_set;
}

void Analysis::IdentifyBagBlobs(std::vector<BlobDescriptor>* blobs) {
	//check every two blobs
	for(unsigned int first_blob_index=0;first_blob_index<blobs->size();first_blob_index++) {
		if((*blobs)[first_blob_index].belongs_to!=-1) continue;
//		if((*blobs)[first_blob_index].first_seen!=frame_number_) continue;
		for(unsigned int second_blob_index=first_blob_index+1;second_blob_index<blobs->size();second_blob_index++) {
			//don't check for multiple ownership
			if((*blobs)[second_blob_index].belongs_to!=-1) continue;
			if((*blobs)[second_blob_index].first_seen!=frame_number_) continue;
			// see if their heights differ
			int first_height = (*blobs)[first_blob_index].last_location.height;
			int second_height = (*blobs)[second_blob_index].last_location.height;

			//if the first one could be the bag
			if(first_height < second_height*0.6) {
				if(utility::AreClose((*blobs)[first_blob_index].last_location,(*blobs)[second_blob_index].last_location)) {
					(*blobs)[first_blob_index].belongs_to = second_blob_index;
				}
			}
			//if the second one could be the bag
			else if (second_height < first_height*0.6) {
				if(utility::AreClose((*blobs)[first_blob_index].last_location,(*blobs)[second_blob_index].last_location)) {
					(*blobs)[second_blob_index].belongs_to = first_blob_index;
				}
			}
			//if they were about the same height
			else {
				continue;
			}
		}
	}
}

/*
 * Name: RegisterBags
 * Description: used to find bags when they are stale in the MOG2 subtraction
 *
 */
void Analysis::RegisterBags(std::vector<BlobDescriptor>* blobs) {
	for(unsigned int blob_index=0;blob_index<(*blobs).size();blob_index++) {
		//check if this blob is a bag and hasn't been seen in 3 frames
		if((*blobs)[blob_index].belongs_to!=NO_OWNER && (*blobs)[blob_index].last_seen < (frame_number_-2)) {
			utility::LookForBagUsingBackProjection(color_raw_, (*blobs)[blob_index]);
		}
	}
}

void Analysis::UpdateAllKalmans(std::vector<BlobDescriptor>* blobs) {
	for(unsigned int i=0;i<(*blobs).size();i++) {
		(*blobs)[i].update_kalman(frame_number_);
	}
}

void Analysis::TrimList(std::vector<BlobDescriptor>* blobs) {
	//get iterator to beginning
	std::vector<BlobDescriptor>::iterator blobs_iterator = (*blobs).begin();
	//search through the list
	for(unsigned int blob_index = 0;blob_index < (*blobs).size();blob_index++) {
		//if the blob was only seen once then delete it
		bool rarely_seen = (*blobs)[blob_index].history.size() == 1;
		bool not_seen_recently = (frame_number_ - (*blobs)[blob_index].last_seen) == 1;
		if(rarely_seen && not_seen_recently) {
			(*blobs).erase(blobs_iterator);
			ROS_INFO_STREAM("Erased the "<<blob_index<<" element");
		}
		blobs_iterator++;
	}
}

void Analysis::UpdateBlobs(std::vector<BlobDescriptor>* known_blobs) {
	std::cout<<"loop"<<std::endl;
	bool debug = false;
	RegionList regions;
	std::vector<BlobDescriptor> blobs;
										if(debug) ROS_INFO_STREAM("Passed step 1");
	GetRegions(&regions);
										if(debug) ROS_INFO_STREAM("Passed step 2");
	CullSmallRegions(&regions);
										if(debug) ROS_INFO_STREAM("Passed step 3");
//	SplitCollidedRegions(*known_blobs,&regions);
										if(debug) ROS_INFO_STREAM("Passed step 4");
	AnalyzeRegions(regions,&blobs);
										if(debug) ROS_INFO_STREAM("Passed step 5");
	RegisterBlobs(blobs,known_blobs);
										if(debug) ROS_INFO_STREAM("Passed step 6");
	IdentifyBagBlobs(known_blobs);
										if(debug) ROS_INFO_STREAM("Passed step 7");
	RegisterBags(known_blobs);
										if(debug) ROS_INFO_STREAM("Passed step 8");
	//update all Kalmans
	UpdateAllKalmans(known_blobs);
										if(debug) ROS_INFO_STREAM("Passed step 9");
	//cull the list
	TrimList(known_blobs);
										if(debug) ROS_INFO_STREAM("Passed step 10");

	//show histograms
	for(int blob_index=0;blob_index<known_blobs->size();blob_index++) {
		std::stringstream name;
		name <<blob_index<<"upper";
		utility::DisplayHistogram(name.str(),(*known_blobs)[blob_index].upper_histogram);
	}
}

/////////////////////// Utilities in this class ///////////////////////////////
/*
 * Name: FindDepthMaskContours
 * Description: allows contours at different depths to be separated instead of
 * 				grouped like they are in the binary image. Not done yet.
 *
 */
void Analysis::FindDepthMaskContours(const cv::Mat& depth_masked,RegionList* regions) {
	//find histogram ranges

	double minVal,maxVal;
	cv::Point minLoc,MaxLoc;
	cv::Mat minmax_mask;
	depth_masked.convertTo(minmax_mask,CV_8UC1);
//	cv::threshold(minmax_mask,minmax_mask,0,255,cv::THRESH_BINARY);
	cv::minMaxLoc(depth_masked,&minVal,&maxVal,&minLoc,&MaxLoc,minmax_mask);
	if(minVal==maxVal) maxVal++;

	int histSize = 20;
	float hist_range[] = {minVal,maxVal};
	const float* ranges = {hist_range};
	cv::MatND histogram;
//	cv::calcHist(&depth_masked,1,0,cv::Mat(),histogram,1,&histSize,&ranges,true,false);
	//TODO: get contours from the histogram values via thresholding
}

/*
 * Name: MakeAMatWithOneBlob
 * Description: extract just one region from the region list and return an
 * 				image which is black except the blob, which is full color
 */
void Analysis::MakeAMatWithOneBlob(const RegionList& regions, const cv::Mat& color_raw, int region_index,cv::Mat* image) {
	//make a binary mask of the interesting region
	cv::Rect bounding_rect = cv::boundingRect(regions[region_index]);
	cv::Mat binary_mask(bounding_rect.height,bounding_rect.width,CV_8UC1,cv::Scalar(0));
	cv::Point offset(0-bounding_rect.x,0-bounding_rect.y);
	cv::drawContours(binary_mask,regions,region_index,cv::Scalar(255),CV_FILLED,8,cv::noArray(),INT_MAX,offset);
	//make a new Mat with the color image (cropped to just the blob)
	cv::Mat color_cropped(color_raw,bounding_rect);
	//use that mask to copy the color image to the resulting return image
	color_cropped.copyTo(*image,binary_mask);
}

/*
 * Name: MakeNewBlobDescriptor
 * Description: Builds a description of a new blob
 */

void Analysis::MakeNewBlobDescriptor(const RegionList& regions, int blob_index, BlobDescriptor* blob) {
	//TODO: do this using smart pointers to avoid reallocations during STL operations
	//fill out the data in the blob descriptor
		//make a Mat with just the color blob
		MakeAMatWithOneBlob(regions,color_raw_,blob_index,&(blob->image));
		//copy the color image
		cv::Size original_size = blob->image.size();
			//get a ROI for this segment of the upper 1/2
			//This does not cause a new data allocation except the header
			cv::Mat upper(blob->image,cv::Rect(0,0,original_size.width,original_size.height*0.5));
			cv::Mat upper_hsv;
			cv::cvtColor(upper,upper_hsv,CV_BGR2HSV);
			//compute the upper histogram
			utility::ComputeHistogram(upper_hsv,&(blob->upper_histogram),utility::MakeBinaryMask(upper));
			//set the ROI for the lower segment to the lower 1/2
			cv::Mat lower(blob->image,cv::Rect(0,original_size.height*0.5,original_size.width,original_size.height*0.5));
			cv::Mat lower_hsv;
			cv::cvtColor(lower,lower_hsv,CV_BGR2HSV);
			//compute the lower histogram
			utility::ComputeHistogram(lower_hsv,&(blob->lower_histogram),utility::MakeBinaryMask(lower));
		//set last seen time to now
		blob->first_seen = blob->last_seen = frame_number_;
		//set last location
		blob->last_location = cv::boundingRect(regions[blob_index]);
		//calculate moment
		blob->moments_ = moments(regions[blob_index],false);

}
