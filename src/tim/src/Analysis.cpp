#include "Analysis.h"

static const int kMinRegionSize=400;
static const int kSameLocationTimeWindow=15;
static const double kMinStrongRelation=0.7;
static const double kMinWeakRelation=0.8;

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
	std::cout<<"Minval: "<<minVal<<"Maxval: "<<maxVal<<std::endl;
	int histSize = 20;
	float hist_range[] = {minVal,maxVal};
	const float* ranges = {hist_range};
	cv::MatND histogram;
	cv::calcHist(&depth_masked,1,0,cv::Mat(),histogram,1,&histSize,&ranges,true,false);
	std::cout<<"Histogram: "<<std::endl<<histogram<<std::endl;
	//TODO: get contours from the histogram values via thresholding
}


//extract just one region from the region list and return an image which is black except the blob, which is full color
void Analysis::MakeAMatWithOneBlob(const RegionList& regions, const cv::Mat& color_raw, int region_index,cv::Mat* image) {
	//make a binary mask of the interesting region
	cv::Mat binary_mask(color_raw.rows,color_raw.cols,CV_8UC1,cv::Scalar(0));
	cv::drawContours(binary_mask,regions,region_index,cv::Scalar(255),CV_FILLED);
	//use that mask to copy the color image to the resulting return image
	color_raw.copyTo(*image,binary_mask);
}

void Analysis::MakeNewBlobDescriptor(const RegionList& regions, int blob_index, BlobDescriptor* blob) {
	//fill out the data in the blob descriptor
		//make a Mat with just the color blob
		cv::Mat this_segment;
		MakeAMatWithOneBlob(regions,color_raw_,blob_index,&this_segment);
		//copy the color image
		cv::Mat temp(this_segment,cv::boundingRect(regions[blob_index]));
		temp.copyTo(blob->image);
		cv::cvtColor(this_segment,this_segment,CV_BGR2HSV);

		cv::Size original_size = this_segment.size();
			//get a ROI for this segment of the upper 2/5.
			//This does not cause a new data allocation except the header
			cv::Mat upper(this_segment,cv::Rect(0,0,original_size.width,original_size.height*0.4));
			std::vector<cv::Mat> upper_planes;
			cv::split(upper,upper_planes);
			//compute the upper histogram
			ComputeHistogram(upper_planes[0],&(blob->upper));
			//set the ROI for this segment to the upper 3/5
			cv::Mat lower(this_segment,cv::Rect(0,original_size.height*0.4,original_size.width,original_size.height*0.6));
			std::vector<cv::Mat> lower_planes;
			cv::split(lower,lower_planes);
			//compute the upper histogram
			ComputeHistogram(lower_planes[0],&(blob->lower));
		//set last seen time to now
		blob->last_seen = blob->first_seen = frame_number_;
		//set last location
		blob->last_location = cv::boundingRect(regions[blob_index]);

		blob->init_kalman();
}

//make a vector of BlobDescriptors for all the significant regions found
void Analysis::AnalyzeRegions(const RegionList& regions,std::vector<BlobDescriptor>* current_blobs) {
	//compute a histogram for each blob and add it to the current blobs
	for(unsigned int blob_index=0;blob_index<regions.size();blob_index++) {
		if(regions[blob_index].size() > kMinRegionSize) {
			cv::Rect bounding_rect = boundingRect(regions[blob_index]);
			//compute the histogram for this blob
			BlobDescriptor this_blob;
			MakeNewBlobDescriptor(regions,blob_index,&this_blob);
			//add the blob to the chain
			current_blobs->push_back(this_blob);
		}
	}
}

void Analysis::ComputeHistogram(const cv::Mat& image, cv::MatND* histogram) {
	int histSize = 180;
	float hist_range[] = {1,180};
	const float* ranges = {hist_range};
	cv::calcHist(&image,1,0,cv::Mat(),*histogram,1,&histSize,&ranges,true,false);
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
		//compare it to each known blob
		for(unsigned int known_blob_index=0;known_blob_index<known_blobs->size() && !blob_set;known_blob_index++) {
			//compare the current and known
				blob_set = RegisterMoving(current_blobs[current_blob_index],&(*known_blobs)[known_blob_index] );

		}
		//if it wasn't updated elsewhere
		if(!blob_set) {
			//otherwise add the blob to the chain
			known_blobs->push_back(current_blobs[current_blob_index]);
		}
		else {
			blob_set = false;
		}
	}
}

bool Analysis::RegisterMoving(const BlobDescriptor& current_blob, BlobDescriptor* known_blob) {
	bool blob_set;
	//if the similarity exceeds a threshold then update it
		//if this blob has already been updated skip it:
		if (current_blob.last_seen == (*known_blob).last_seen) {
			return false;
		}
		//compare the last updated time to now
		assert(current_blob.last_seen > (*known_blob).last_seen);
		if((current_blob.last_seen-(*known_blob).last_seen) < kSameLocationTimeWindow){
			//if it was updated recently then check if the new center is within the old center
			int new_x = current_blob.last_location.x,
			    new_y = current_blob.last_location.y,
			    new_width = current_blob.last_location.width,
			    new_height = current_blob.last_location.height;

			cv::Point new_center(new_x+new_width/2,new_y+new_height/2);
			if((*known_blob).last_location.contains(new_center)) {
				//schedule the blob to be updated
				//TODO: can't just update it otherwise it could thrash or something
				//But I'm going to do it anyway for now
				double relation = CompareBlobs((*known_blob), current_blob);
				if(relation > kMinStrongRelation) {
					(*known_blob).update(current_blob);
				}
				blob_set = true;
			}
			else return false; // the blob moved too much
		}
		else {
			//getting here means the blob was possibly occluded a while ago and now re-visible
			//don't need to check location, just strong similarity
			double relation = CompareBlobs((*known_blob), current_blob);
			if(relation > kMinWeakRelation) {
				(*known_blob).update(current_blob);
				blob_set = true;
			}
			else return false; //the blob is too unalike
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
			std::stringstream text;
			text<<"back project "<<blob_index;
			imshow(text.str(),bag_contour_mat);
		}
	}
}

double Analysis::CompareBlobs(const BlobDescriptor& first, const BlobDescriptor& second) {
	double relation = 0;
	relation += compareHist(first.upper,second.upper,CV_COMP_CORREL);
	relation += compareHist(first.lower,second.lower,CV_COMP_CORREL);
	relation /= 2;
	return relation;
}

void Analysis::UpdateBlobs(std::vector<BlobDescriptor>* known_blobs) {
	RegionList regions;
	std::vector<BlobDescriptor> blobs;
	GetRegions(&regions);
	AnalyzeRegions(regions,&blobs);
	RegisterBlobs(blobs,known_blobs);
	LookForBags(known_blobs);
	RegisterBags(known_blobs);

	//update all Kalmans
	UpdateAllKalmans(known_blobs);
}

void Analysis::LookForBags(std::vector<BlobDescriptor>* blobs) {
	//check every two blobs
	for(unsigned int first_blob_index=0;first_blob_index<blobs->size();first_blob_index++) {
		if((*blobs)[first_blob_index].belongs_to!=-1) continue;
		if((*blobs)[first_blob_index].first_seen!=frame_number_) continue;
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
		(*blobs)[i].update_kalman();
	}
}
