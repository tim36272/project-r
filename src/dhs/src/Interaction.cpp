/*
 * Interaction.cpp
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

#include "dhs/Interaction.h"

namespace interaction {
void checkForInteractions(BlobDescriptorDecoratedKBPtr first_blob, const std::vector<BlobPtr>& other_blobs, Interactions& interactions) {
	if(!first_blob->bag()) {
		/*
		 * Single Person
		 */
		InteractionStatePtr single_person(new InteractionState);
		if(checkForSinglePerson(first_blob,single_person)) {
			//This will either insert, or not if it already exists
			std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(single_person->getHash(),single_person));
			if(!inserted.second)
				inserted.first->second->add_seen_at(first_blob->lastSeen());
		}
		/*
		 * Two blob events need a second blob
		 */
		std::vector<BlobPtr>::const_iterator second_blob_it = other_blobs.begin();
		while(second_blob_it!=other_blobs.end()) {
			if(*second_blob_it == first_blob) {
				++second_blob_it;
				continue;
			}
			/*
			 * Bag interaction
			 */
			InteractionStatePtr bag_abandoned(new InteractionState);
			if(checkForBagAbandoned(first_blob,*second_blob_it,bag_abandoned)) {
				//This will either insert, or not if it already exists
				std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(bag_abandoned->getHash(),bag_abandoned));
				if(!inserted.second)
					inserted.first->second->add_seen_at(first_blob->lastSeen());
			}
			/*
			 * Two Person meeting
			 */
			InteractionStatePtr two_person_meeting(new InteractionState);
			if(checkForTwoPeopleMeeting(first_blob,*second_blob_it,two_person_meeting)) {
				//This will either insert, or not if it already exists
				std::pair<Interactions::iterator,bool> inserted = interactions.insert(std::pair<int,InteractionStatePtr>(two_person_meeting->getHash(),two_person_meeting));
				if(!inserted.second)
					inserted.first->second->add_seen_at(first_blob->lastSeen());
			}
			++second_blob_it;
		}
	}
	else { //it's a bag

	}
}

bool checkForSinglePerson(const BlobDescriptorDecoratedKBPtr blob, interaction::InteractionStatePtr interaction) {
	if(blob->bag()) return false;

	interaction->set_agent_1_id(blob->Id());
	interaction->set_agent_2_id(interaction::kNotSet);
	interaction->set_bag_id(interaction::kNotSet);
	interaction->add_seen_at(blob->lastSeen());
	interaction->set_interaction(InteractionState::single_person);
	//this event is always relevant
	return true;
}

bool checkForTwoPeopleMeeting(const BlobDescriptorDecoratedKBPtr first_blob, const BlobDescriptorDecoratedKBPtr second_blob, InteractionStatePtr interaction) {
	//must both be visible at the same time
	if(first_blob->lastSeen() != second_blob->lastSeen()) return false;
	if(first_blob->bag() || second_blob->bag()) return false;

	//TODO: add depth checking
	//check if they are close to eachother
	int separation =  utility::distance(
						utility::Center(first_blob->getLastFilteredBound()),
						utility::Center(second_blob->getLastFilteredBound()));
	int max_separation = first_blob->getLastFilteredBound().x + second_blob->getLastFilteredBound().x;

	if(separation < max_separation) {
		interaction->set_agent_1_id(first_blob->Id());
		interaction->set_agent_2_id(second_blob->Id());
		interaction->set_bag_id(interaction::kNotSet);
		interaction->add_seen_at(first_blob->lastSeen());
		interaction->set_interaction(InteractionState::two_person_meeting);
		return true;
	}
	return false;
}
bool checkForBagAbandoned(BlobDescriptorDecoratedKBPtr first_blob, const BlobDescriptorDecoratedKBPtr second_blob, InteractionStatePtr interaction) {
	if()
}

void print(Interactions interactions) {
	std::cout<<"Blob history: ";
	interaction::Interactions::iterator interaction_it = interactions.begin();
	while(interaction_it != interactions.end()) {
		std::cout<<"Blobs: "
				 <<interaction_it->second->agent_1_id_<<","
				 <<interaction_it->second->agent_2_id_
				 <<" are having interaction type "
				 <<interaction_it->second->interaction_<<": ";
		std::vector<std::pair<int,int> >::iterator event_it = interaction_it->second->instances_.begin();
		while(event_it!=interaction_it->second->instances_.end()) {
			std::cout<<"("<<event_it->first<<","<<event_it->second<<")";
			++event_it;
		}
		std::cout<<std::endl;
		++interaction_it;
	}
	std::cout<<std::endl;
}

}; // namespace interaction
