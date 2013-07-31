#include "Analysis.h"

#define HISTOGRAM_COMPARE_METHOD /*CV_COMP_CHISQR*/ CV_COMP_CHISQR

static const int kMinRegionSize=400;
static const int kSameLocationTimeWindow=5;
static const double kMinChisqrStrongRelation=5000;
static const double kMinCorrelStrongRelation=0.7;
static const double kMinIntersectStrongRelation=15;
static const double kMinBhattacharyyaStrongRelation=0.2;
static const double kMinChisqrWeakRelation=6000;
static const double kMinCorrelWeakRelation=0.5;
static const double kMinIntersectWeakRelation=4;
static const double kMinBhattacharyyaWeakRelation=0.6;


static const double kMinWeakRelation=0.6;

Analysis::Analysis() {
	frame_number_ = 0;
	color_segmentation_.set("nmixtures",3);
	depth_segmentation_.set("nmixtures",3);
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
		//smoothing

	//update the depth segmentation
		depth_segmentation_(depth_raw,depth_foreground_);
		//smoothing

	//create a unified foreground
		combined_foreground_ = depth_foreground_ & color_foreground_;

	return frame_number_;
}

//passes back a RegionList of interesting points from the segmentation
void Analysis::GetRegions(RegionList* regions) {
	//use combined foreground to mask depth image to src
	cv::Mat depth_masked;
	depth_raw_.copyTo(depth_masked,combined_foreground_);
	RegionList temp_regions;
	double minVal,maxVal;
//	depth_masked.convertTo(depth_masked,CV_8UC1);
	FindDepthMaskContours(depth_masked,&temp_regions);

	imshow("depth masked",depth_masked);

	//cv::findContours(depth_masked,*regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	cv::findContours(combined_foreground_,*regions,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
}

void Analysis::FindDepthMaskContours(const cv::Mat& depth_masked,RegionList* regions) {
	//find histogram ranges

	double minVal,maxVal;
	cv::Point minLoc,MaxLoc;
	cv::Mat minmax_mask;
	depth_masked.convertTo(minmax_mask,CV_8UC1);
	cv::threshold(minmax_mask,minmax_mask,0,255,cv::THRESH_BINARY);
	cv::minMaxLoc(depth_masked,&minVal,&maxVal,&minLoc,&MaxLoc,minmax_mask);
//	std::cout<<"Minval: "<<minVal<<"Maxval: "<<maxVal<<std::endl;
	int histSize = 20;
	float hist_range[] = {minVal,maxVal};
	const float* ranges = {hist_range};
	cv::MatND histogram;
	cv::calcHist(&depth_masked,1,0,cv::Mat(),histogram,1,&histSize,&ranges,true,false);
//	std::cout<<"Histogram: "<<std::endl<<histogram<<std::endl;
	//TODO: get contours from the histogram values via thresholding
}


//extract just one region from the region list and return an image which is black except the blob, which is full color
void Analysis::MakeAMatWithOneBlob(const RegionList& regions, const cv::Mat& color_raw, int region_index,cv::Mat* image) {
	//make a binary mask of the interesting region
	cv::Rect bounding_rect = cv::boundingRect(regions[region_index]);
	cv::Mat binary_mask(bounding_rect.height,bounding_rect.width,CV_8UC1,cv::Scalar(0));
	cv::Point offset(0-bounding_rect.x,0-bounding_rect.y);
	std::cout<<"TS_CP1"<<std::endl;
	cv::drawContours(binary_mask,regions,region_index,cv::Scalar(255),CV_FILLED,8,cv::noArray(),INT_MAX,offset);
	std::cout<<"TS_CP2"<<std::endl;
	//make a new Mat with the color image (cropped to just the blob)
	cv::Mat color_cropped(color_raw,bounding_rect);
	std::cout<<"TS_CP3"<<std::endl;
	//use that mask to copy the color image to the resulting return image
	color_cropped.copyTo(*image,binary_mask);
	std::cout<<"TS_CP4"<<std::endl;
	imshow("*image",*image);
	std::cout<<"TS_CP5"<<std::endl;
}

void Analysis::MakeNewBlobDescriptor(const RegionList& regions, int blob_index, BlobDescriptor* blob) {
	//TODO: do this using smart pointers to avoid reallocations during STL operations
	//fill out the data in the blob descriptor
		//make a Mat with just the color blob
	cv::Mat this_segment_hsv;
		MakeAMatWithOneBlob(regions,color_raw_,blob_index,&(blob->image));
		//copy the color image
		cv::cvtColor(blob->image,this_segment_hsv,CV_BGR2HSV);

		cv::Size original_size = this_segment_hsv.size();
			//get a ROI for this segment of the upper 2/5.
			//This does not cause a new data allocation except the header
			cv::Mat upper(this_segment_hsv,cv::Rect(0,0,original_size.width,original_size.height*0.5));
			std::vector<cv::Mat> upper_planes;
			cv::split(upper,upper_planes);
			upper_planes[0].copyTo(blob->upper_hue_plane);
			//compute the upper histogram
			ComputeHistogram(upper_planes[0],&(blob->upper));
			//set the ROI for this segment to the upper 3/5
			cv::Mat lower(this_segment_hsv,cv::Rect(0,original_size.height*0.5,original_size.width,original_size.height*0.5));
			std::vector<cv::Mat> lower_planes;
			cv::split(lower,lower_planes);
			lower_planes[0].copyTo(blob->lower_hue_plane);

			//compute the upper histogram
			ComputeHistogram(lower_planes[0],&(blob->lower));

			//save the hue planes for diagnostics

		//set last seen time to now
		blob->last_seen = blob->first_seen = frame_number_;
		//set last location
		blob->last_location = cv::boundingRect(regions[blob_index]);
}

//make a vector of BlobDescriptors for all the significant regions found
void Analysis::AnalyzeRegions(const RegionList& regions,std::vector<BlobDescriptor>* current_blobs) {
	//compute a histogram for each blob and add it to the current blobs
	cv::Mat all_contours(color_raw_.size(),CV_8UC3,cv::Scalar(0));
	for(unsigned int blob_index=0;blob_index<regions.size();blob_index++) {
		//require a certain size and certain density
		if(regions[blob_index].size() > kMinRegionSize) {
			cv::Rect bounding_rect = boundingRect(regions[blob_index]);
			//compute the histogram for this blob
			BlobDescriptor this_blob;
			MakeNewBlobDescriptor(regions,blob_index,&this_blob);
			//add the blob to the chain
			current_blobs->push_back(this_blob);

			//make a Mat of each contour
			cv::drawContours(all_contours,regions,blob_index,cv::Scalar(rand()%255,rand()%255,rand()%255),-1,8);
		}
	}
	imshow("All contours",all_contours);
}

void Analysis::ComputeHistogram(const cv::Mat& image, cv::MatND* histogram) {
	int histSize = 180;
	float hist_range[] = {1,180};
	const float* ranges = {hist_range};
	cv::calcHist(&image,1,0,cv::Mat(),*histogram,1,&histSize,&ranges,true,false);
//	cv::normalize(*histogram,*histogram,0,histogram->rows,cv::NORM_MINMAX,-1,cv::Mat());
}

void Analysis::ComputeBackProjection( const BlobDescriptor& blob , cv::MatND* back_projection ) {
	cv::Mat color_hsv;
	cv::cvtColor(color_raw_,color_hsv,CV_BGR2HSV);
	std::vector<cv::Mat> planes;
	cv::split(color_hsv,planes);
	cv::Mat mask1,mask2;
	//make a mask to suppress low saturation
	cv::threshold(planes[1],mask1,60,255,cv::THRESH_BINARY);
	//make a mask to suppress bright pixels
	cv::threshold(planes[2],mask2,170,255,cv::THRESH_BINARY_INV);
	mask1 = mask1 & mask2;
	cv::Mat hue;
	planes[0].copyTo(hue,mask1);
	cv::MatND upper_back_projection,lower_back_projection;
	float hist_range[] = {1,180};
	const float* ranges = {hist_range};
	cv::calcBackProject(&hue,1,0,blob.upper,upper_back_projection,&ranges,1);
	cv::calcBackProject(&hue,1,0,blob.lower,lower_back_projection,&ranges,1);
	*back_projection = upper_back_projection | lower_back_projection;
}

//takes a set of histograms and match them to the known set or add them to the set, updating describing information
void Analysis::RegisterBlobs(const std::vector<BlobDescriptor>& current_blobs, std::vector<BlobDescriptor>* known_blobs) {

	bool blob_set = false;
	//for each current blob
	for(unsigned int current_blob_index=0;current_blob_index<current_blobs.size();current_blob_index++) {
		//compare it to each known blobs
		for(unsigned int known_blob_index=0;known_blob_index<known_blobs->size() && !blob_set;known_blob_index++) {
			//compare the current and known
			std::cout<<"Checking current "<<current_blob_index<<" against known "<<known_blob_index<<std::endl;
				blob_set = RegisterMoving(current_blobs[current_blob_index],&((*known_blobs)[known_blob_index]) );
/*				if(blob_set) {
					//report that a blob was matched
					std::cout<<"current "<<current_blob_index<<"matched to known "
							 <<known_blob_index<<current_blobs[current_blob_index].last_location<<" , "
							 <<known_blob_index<<(*known_blobs)[known_blob_index].last_location<<std::endl;
				}
				else {
					//dump the 0'th and current images
					cv::imwrite("current_image.png",current_blobs[current_blob_index].image);
					cv::imwrite("known_image.png",(*known_blobs)[0].image);
					cv::imwrite("current_upper_hue.png",current_blobs[current_blob_index].upper_hue_plane);
					cv::imwrite("known_upper_hue.png",(*known_blobs)[0].upper_hue_plane);
					cv::imwrite("current_upper_hist.png",current_blobs[current_blob_index].upper);
					cv::imwrite("known_upper_hist.png",(*known_blobs)[0].upper);
					std::cout<<"image writing comparison:"<<std::endl;
					std::cout<<current_blobs[current_blob_index].upper<<std::endl<<std::endl<<(*known_blobs)[0].upper<<std::endl;
				}*/
		}
		//if it wasn't updated elsewhere
		if(!blob_set) {
			//if it wasn't updated then add the blob to the chain
			known_blobs->push_back(current_blobs[current_blob_index]);
			known_blobs->back().init_kalman();
		}
		else {
			//otherwise move on to the next blob
			blob_set = false;
		}
	}
}

bool Analysis::RegisterMoving(const BlobDescriptor& current_blob, BlobDescriptor* known_blob) {
	bool blob_set = false;
	//if the similarity exceeds a threshold then update it
		//if this blob has already been updated skip it:
		if (current_blob.last_seen == (*known_blob).last_seen) {
			std::cout<<"already updated"<<std::endl;
			return false;
		}
		//compare the last updated time to now
		assert(current_blob.last_seen > (*known_blob).last_seen);
		if((current_blob.last_seen-(*known_blob).last_seen) < kSameLocationTimeWindow){
			//if it was updated recently then check if it is where the Kalman filter expects it to be

			cv::Point new_center = utility::calcCenter(current_blob.last_location);
			cv::Point kalman_center = utility::calcCenter((*known_blob).filter.bounding_rect_estimate());

			bool kalman_prediction_contains_new_center = (*known_blob).filter.bounding_rect_estimate().contains(new_center);
			bool new_blob_contains_kalman_prediction_center = current_blob.last_location.contains(kalman_center);

			if(kalman_prediction_contains_new_center || new_blob_contains_kalman_prediction_center) {
				//check if this blob is almost completely contained within the known blob
					//find the overlapping area

					int shared_area = utility::CalculateSharedArea(known_blob->filter.bounding_rect_estimate(), current_blob.last_location);
					std::cout<<"shared area: "<<shared_area<<" current area: "<<current_blob.last_location.area()<<" known area: "<<known_blob->filter.bounding_rect_estimate().area()<<std::endl;
					std::cout<<"Current rect: "<<current_blob.last_location<<" known rect: "<<known_blob->filter.bounding_rect_estimate()<<std::endl;
					if( shared_area > current_blob.last_location.area()/2 && current_blob.last_location.area() < known_blob->filter.bounding_rect_estimate().area()/2) {
						//we got a fragment of the blob, so move on with life
						return true;
					}
				//see if the blob is similar enough to be updated
				double relation = CompareBlobs((*known_blob), current_blob);
				if(CheckForStrongRelation(relation)) {
					(*known_blob).update(current_blob);
					std::cout<<"same: "<<relation<<std::endl;
					blob_set = true;
//					std::cout<<"Upper same comparison: "<<std::endl;
//					utility::PrintHistogramComparison((*known_blob).upper,current_blob.upper);
//					std::cout<<"lower same comparison: "<<std::endl;
//					utility::PrintHistogramComparison((*known_blob).lower,current_blob.lower);
				}
				else {
					//there must be something occluding this blob because it is way different than what was expected since it was in the same place
					std::cout<<"different: "<<relation<<std::endl;
//					std::cout<<"Upper different comparison: "<<std::endl;
//					utility::PrintHistogramComparison((*known_blob).upper,current_blob.upper);
//					std::cout<<"lower different comparison: "<<std::endl;
//					utility::PrintHistogramComparison((*known_blob).lower,current_blob.lower);
					return false;
				}
			}
			else {
				std::cout<<"moved"<<std::endl;
				return false; // the blob is in a totally different place
			}
		}
		else {
			std::cout<<"Been a while"<<std::endl;
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
					double relation = CompareBlobs((*known_blob), current_blob);
					if(relation > kMinWeakRelation) {
						(*known_blob).update(current_blob);
						blob_set = true;
						std::cout<<"the same"<<std::endl;
					}
					else {
						std::cout<<"dissimilar"<<std::endl;
						return false; // the blob is really dissimilar
					}
				}
				else {
					std::cout<<"moved"<<std::endl;
					return false; // it is not where it should be
				}
			}
	return blob_set;
}

void Analysis::RegisterBags(std::vector<BlobDescriptor>* blobs) {
	for(unsigned int blob_index=0;blob_index<(*blobs).size();blob_index++) {
		if((*blobs)[blob_index].belongs_to!=NO_OWNER) {
			cv::MatND histogram;
			ComputeBackProjection( (*blobs)[blob_index] , &histogram );
			//get contours from the back projection
			cv::Mat bag_foreground;
			cv::threshold(histogram,bag_foreground,128,255,cv::THRESH_BINARY);
			cv::dilate(bag_foreground,bag_foreground,cv::Mat());
			cv::erode(bag_foreground,bag_foreground,cv::Mat());
			std::vector<std::vector<cv::Point> > bag_contours;
			cv::findContours(bag_foreground,bag_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
			cv::Mat bag_contour_mat(bag_foreground.size(),CV_8UC3,cv::Scalar(0));
			for(unsigned int i=0;i<bag_contours.size();i++) {
				if(bag_contours[i].size()>kMinRegionSize) {
					//color them and encode their region number into blue and green
					int blue = i%256;
					int green = (i/256)%256;
					cv::Scalar color(blue,green,rand()%256);
					//draw the contour on the Mat
					cv::drawContours(bag_contour_mat,bag_contours,i,color,-1);
				}
			}
		}
	}
}

double Analysis::CompareBlobs(const BlobDescriptor& first, const BlobDescriptor& second) {
	double relation = 0;
	if(HISTOGRAM_COMPARE_METHOD == CV_COMP_CHISQR) {
		relation += compareHist(first.upper,second.upper,CV_COMP_CHISQR);
		relation += compareHist(first.lower,second.lower,CV_COMP_CHISQR);
	}
	else if (HISTOGRAM_COMPARE_METHOD == CV_COMP_CORREL) {
		relation += compareHist(first.upper,second.upper,CV_COMP_CORREL);
		relation += compareHist(first.lower,second.lower,CV_COMP_CORREL);
	}
	else if (HISTOGRAM_COMPARE_METHOD== CV_COMP_INTERSECT){
		relation += compareHist(first.upper,second.upper,CV_COMP_INTERSECT);
		relation += compareHist(first.lower,second.lower,CV_COMP_INTERSECT);
	}
	else {
		relation += compareHist(first.upper,second.upper,CV_COMP_BHATTACHARYYA);
		relation += compareHist(first.lower,second.lower,CV_COMP_BHATTACHARYYA);
	}
	relation /= 2;
	return relation;
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
			std::cout<<"Erased the "<<blob_index<<"element"<<std::endl;
		}
		blobs_iterator++;
	}
}

void Analysis::UpdateBlobs(std::vector<BlobDescriptor>* known_blobs) {
	bool debug = false;
	RegionList regions;
	std::vector<BlobDescriptor> blobs;
			if(debug) std::cout<<"1"<<std::endl;
	GetRegions(&regions);
			if(debug) std::cout<<"2"<<std::endl;
	AnalyzeRegions(regions,&blobs);
			if(debug) std::cout<<"3"<<std::endl;
	RegisterBlobs(blobs,known_blobs);
			if(debug) std::cout<<"4"<<std::endl;
	LookForBags(known_blobs);
			if(debug) std::cout<<"5"<<std::endl;
	RegisterBags(known_blobs);
			if(debug) std::cout<<"6"<<std::endl;
	//update all Kalmans
	UpdateAllKalmans(known_blobs);
			if(debug) std::cout<<"7"<<std::endl;
	//cull the list
	TrimList(known_blobs);
			if(debug) std::cout<<"8"<<std::endl;
}

void Analysis::LookForBags(std::vector<BlobDescriptor>* blobs) {
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
				if(AreClose((*blobs)[first_blob_index].last_location,(*blobs)[second_blob_index].last_location)) {
					(*blobs)[first_blob_index].belongs_to = second_blob_index;
				}
			}
			//if the second one could be the bag
			else if (second_height < first_height*0.6) {
				if(AreClose((*blobs)[first_blob_index].last_location,(*blobs)[second_blob_index].last_location)) {
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

bool Analysis::AreClose(const cv::Rect& lhs, const cv::Rect& rhs) {
	cv::Point lhs_center(lhs.x+lhs.width/2,lhs.y+lhs.height/2);
	cv::Point rhs_center(rhs.x+rhs.width/2,rhs.y+rhs.height/2);
	int distance = pow(lhs_center.x-rhs_center.x,2)+pow(lhs_center.y-rhs_center.y,2);
	distance = sqrt(distance);
	if(distance < (lhs.width+rhs.width)) return true;
	else return false;
}

void Analysis::UpdateAllKalmans(std::vector<BlobDescriptor>* blobs) {
	for(unsigned int i=0;i<(*blobs).size();i++) {
		(*blobs)[i].update_kalman(frame_number_);
	}
}

bool Analysis::CheckForStrongRelation(double relation) {
	if(HISTOGRAM_COMPARE_METHOD == CV_COMP_CHISQR) {
		return relation < kMinChisqrStrongRelation;
	}
	else if (HISTOGRAM_COMPARE_METHOD == CV_COMP_CORREL) {
		return relation > kMinCorrelStrongRelation;
	}
	else if (HISTOGRAM_COMPARE_METHOD== CV_COMP_INTERSECT){
		return relation > kMinIntersectStrongRelation;
	}
	else {
		return relation < kMinBhattacharyyaStrongRelation;
	}
}

bool Analysis::CheckForWeakRelation(double relation) {
	if(HISTOGRAM_COMPARE_METHOD == CV_COMP_CHISQR) {
		return relation < kMinChisqrWeakRelation;
	}
	else if (HISTOGRAM_COMPARE_METHOD == CV_COMP_CORREL) {
		return relation > kMinCorrelWeakRelation;
	}
	else if (HISTOGRAM_COMPARE_METHOD== CV_COMP_INTERSECT){
		return relation > kMinIntersectWeakRelation;
	}
	else {
		return relation < kMinBhattacharyyaWeakRelation;
	}
}
