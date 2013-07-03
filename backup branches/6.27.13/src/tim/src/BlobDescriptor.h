#ifndef TIM_SRC_BLOBDESCRIPTOR_H
#define TIM_SRC_BLOBDESCRIPTOR_H

#define NO_OWNER -1
struct BlobDescriptor {
	cv::MatND upper,lower;
	int last_seen;
	int first_seen;
	cv::Rect last_location;
	cv::Mat image;
	int belongs_to;
	BlobDescriptor() {
		last_seen=first_seen=-1;
		belongs_to = NO_OWNER;
		last_location = cv::Rect();
	}
	BlobDescriptor(const BlobDescriptor& rhs) {
		image = rhs.image;
		last_seen = rhs.last_seen;
		first_seen = rhs.first_seen;
		last_location = rhs.last_location;
		rhs.upper.copyTo(upper);
		rhs.lower.copyTo(lower);
		belongs_to = rhs.belongs_to;
	}
	BlobDescriptor& operator=(const BlobDescriptor& rhs) {
		image = rhs.image;
		last_seen = rhs.last_seen;
		first_seen = rhs.first_seen;
		last_location = rhs.last_location;
		upper = rhs.upper;
		lower = rhs.lower;
		belongs_to = rhs. belongs_to;
		return *this;
	}
	std::ostream& operator << (std::ostream& out) {
		out <<"First seen: "<<first_seen<<"Last seen: "<<last_seen<<" at: "<<last_location<<" belongs to: "<<belongs_to;
		return out;
	}
	void update(const BlobDescriptor& rhs) {
		image = rhs.image;
		last_seen = rhs.last_seen;
		//don't update first_seen
		last_location = rhs.last_location;
		upper = rhs.upper;
		lower = rhs.lower;
//		upper = upper*0.5+rhs.upper*0.5;
//		lower = lower*0.5+rhs.lower*0.5;
		belongs_to = rhs.belongs_to;
	}
};
#endif
