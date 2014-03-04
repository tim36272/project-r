/*
 * InteractionState.h
    Copyright (C) 2014 Timothy Sweet

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INTERACTIONSTATE_H_
#define INTERACTIONSTATE_H_

#include <vector>
#include <map>
#include <boost/smart_ptr/shared_ptr.hpp>

#include "dhs/Utility.h"
#include "dhs/BlobDescriptorDecorated.h"

namespace interaction {
typedef BlobDescriptorDecoratedKBPtr BlobPtr;
static const int kNotSet(-1);
struct InteractionState {
	enum InteractionType {
		single_person,
		bag_abandon,
		bag_steal,
		bag_exchange,
		two_person_meeting
	};
	InteractionType interaction_;
	int agent_1_id_,agent_2_id_,bag_id_;
	std::vector<std::pair<int,int> > instances_;
	void set_agent_1_id(int id){ agent_1_id_ = id;}
	void set_agent_2_id(int id){ agent_2_id_ = id;}
	void set_bag_id(int id){ bag_id_ = id;}
	void set_interaction(InteractionType interaction) {interaction_ = interaction;}
	void add_seen_at(int time) {
		if(instances_.size()==0) {
			//event has never been seen
			instances_.push_back(std::pair<int,int>(time,time));
			return;
		}
		int last_seen = instances_.rbegin()->second;
		if(last_seen >= time-30) {
			//this is a continuation of the existing instance
			instances_.rbegin()->second = time;
		} else {
			//this is a new instance
			instances_.push_back(std::pair<int,int>(time,time));
		}
		return;
	}



	int getHash() {
		int hash_factor=1, hash_multiplier=10;
		int hash=0;
		hash += agent_1_id_ * hash_factor;
		hash_factor *= hash_multiplier;
		hash += agent_2_id_ * hash_factor;
		hash_factor *= hash_multiplier;
		hash += bag_id_ * hash_factor;
		hash_factor *= hash_multiplier;
		hash += interaction_ * hash_factor;
		hash_factor *= hash_multiplier;
		return hash;

	}
};

typedef boost::shared_ptr<InteractionState> InteractionStatePtr;
typedef std::map<int,InteractionStatePtr> Interactions;

//calls each of the checkFor's which causes interactions to be populated
void checkForInteractions(const BlobDescriptorDecoratedKBPtr first_blob, const std::vector<BlobPtr>& other_blobs, Interactions& interactions);
//populates interaction if that type is detected
bool checkForSinglePerson(const BlobDescriptorDecoratedKBPtr blob, InteractionStatePtr interaction);
bool checkForTwoPeopleMeeting(const BlobDescriptorDecoratedKBPtr first_blob, const BlobDescriptorDecoratedKBPtr second_blob, InteractionStatePtr interaction);

void print(Interactions interactions);

}; /* namespace interaction */
#endif /* INTERACTIONSTATE_H_ */
