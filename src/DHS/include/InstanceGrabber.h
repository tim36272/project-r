#ifndef INSTANCEGRABBER_H_
#define INSTANCEGRABBER_H_

#include <vector>
#include <fstream>
#include <ros/console.h>
#include "utility.h"

#define EVENT_ONE_PERSON_SEARCH 3
#define EVENT_ONE_PERSON_ABANDON 4
#define EVENT_TWO_PERSON_SEARCH 5
#define EVENT_TWO_PERSON_STEAL 6
#define EVENT_TWO_PERSON_EXCHANGE 7

#define START_TIME 0
#define END_TIME 1

#define UPPER_COLOR 0
#define LOWER_COLOR 1
#define NOT_FOUND -1

class InstanceGrabber {
public:
	InstanceGrabber();
	InstanceGrabber(uint objective_code,
			 const cv::Scalar person_one_upper_color,
			 const cv::Scalar person_one_lower_color,
			 const cv::Scalar  person_two_upper_color,
			 const cv::Scalar  person_two_lower_color,
			 const cv::Scalar bag_color,
			 int person_one_source_index,
			 int person_two_source_index,
			 int bag_source_index);
	void Setup(uint objective_code,
			 const cv::Scalar person_one_upper_color,
			 const cv::Scalar person_one_lower_color,
			 const cv::Scalar  person_two_upper_color,
			 const cv::Scalar  person_two_lower_color,
			 const cv::Scalar bag_color,
			 int person_one_source_index,
			 int person_two_source_index,
			 int bag_source_index);


	void Update(int frame_number,const PersonList& people,const BagList& bags);
	friend std::ostream& operator <<(std::ostream& out, const InstanceGrabber& rhs);
	bool EventInProgress() const;
	int event_code() const {return event_code_;}
	std::vector<utility::Pair > Instances() {return instances_;}
	int person_one_source() const {return person_one_source_index_;}
	int person_two_source() const {return person_two_source_index_;}
	int bag_source() const {return bag_source_index_;}
	utility::Pair operator [](int index) {return instances_[index];}


private:
	bool looking_for_start_,setup_done_;
	int event_code_,frame_number_,missing_person_buffer_;
public:
	int person_one_source_index_,person_two_source_index_,bag_source_index_;
private:
	cv::Scalar bag_color_,person_one_upper_color_,person_one_lower_color_,person_two_upper_color_,person_two_lower_color_;
public:
	std::vector<utility::Pair > instances_;
private:
	cv::Rect bag_location_;
	int GetFirstPersonIndex(const PersonList& people);
	int GetSecondPersonIndex(const PersonList& people);

	int GetBagIndex(const BagList& bags);


	void UpdateOnePersonSearch(const PersonList& people);
	void UpdateOnePersonAbandon(const PersonList& people,const BagList& bags);
	void UpdateTwoPersonSearch(const PersonList& people);
	void UpdateTwoPersonSteal(const PersonList& people,const BagList& bags);
	void UpdateTwoPersonExchange(const PersonList& people,const BagList& bags);
};

#endif /* INSTANCEGRABBER_H_ */
