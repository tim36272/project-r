#ifndef INSTANCEGRABBER_H_
#define INSTANCEGRABBER_H_

#include <vector>
#include <fstream>
#include <ros/console.h>
#include "utilities.cpp"

#define R_NO_EVENT					-1
#define R_ONE_PERSON_TRACK	 		0
#define R_ONE_PERSON_BAG_UNATTENDED	1
#define R_TWO_PERSON_MEET 			2
#define R_TWO_PERSON_BAG_STEAL 		3
#define R_TWO_PERSON_BAG_EXCHANGE 	4

#define START_TIME 0
#define END_TIME 1

class InstanceGrabber {
public:
	InstanceGrabber();
	InstanceGrabber(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower);
	InstanceGrabber(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower,
					const cv::Scalar& bag);

	InstanceGrabber(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower,
					const cv::Scalar& person_2_upper,
					const cv::Scalar& person_2_lower);

	InstanceGrabber(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower,
					const cv::Scalar& person_2_upper,
					const cv::Scalar& person_2_lower,
					const cv::Scalar& bag);

	void Update(int frame_number,const PersonList& people,const BagList& bags);
	void Update(int frame_number,const PersonList& people);
	void Setup(int objective_code,
			const cv::Scalar& person_1_upper,
			const cv::Scalar& person_1_lower);
	void Setup(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower,
					const cv::Scalar& bag);
	void Setup(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower,
					const cv::Scalar& person_2_upper,
					const cv::Scalar& person_2_lower);
	void Setup(int objective_code,
					const cv::Scalar& person_1_upper,
					const cv::Scalar& person_1_lower,
					const cv::Scalar& person_2_upper,
					const cv::Scalar& person_2_lower,
					const cv::Scalar& bag);

	void PrintInstances(std::ostream& out);
	bool EventInProgress() const;
	bool AlreadyStolen() const;
	std::vector<utility::Pair_<uint> > Instances() {return instances_;}

private:
	bool looking_for_start_,setup_done_;
	int objective_code_,frame_number_,missing_person_buffer_;
	cv::Scalar person_1_upper_,person_1_lower_,person_2_upper_,person_2_lower_,bag_;
public:
	std::vector<utility::Pair_<uint> > instances_;
private:
	cv::Rect bag_location_;
	bool already_stolen_;
	void UpdateOnePersonTrack(const PersonList& people);
	void UpdateOnePersonBagUnattended(const PersonList& people,const BagList& bags);
	void UpdateTwoPersonBagExchange(const PersonList& people,const BagList& bags);
	void UpdateTwoPersonBagSteal(const PersonList& people,const BagList& bags);
	void UpdateTwoPersonMeet(const PersonList& people);
};

#endif /* INSTANCEGRABBER_H_ */
