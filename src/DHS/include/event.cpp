#ifndef SRC_INCLUDE_EVENT_CPP
#define SRC_INCLUDE_EVENT_CPP
static const uint kMenuWidth = 350;
static const cv::Scalar kFontColor(255,255,255);
static const std::string kMenuName = "Menu";
static const uint kMenuItemHeight = 20;
static const uint kMenuFontThickness = 1;
static const double kMenuFontScale = 0.5;


#define NO_EVENT -1
#define EVENT_ANY 254
#define EVENT_NOT_USED 253
#define MENU_ONE_PERSON 1
#define MENU_TWO_PERSON 2
#define MENU_ALL_EVENTS 3

#define MENU_ONE_PERSON_SEARCH 1
#define MENU_ONE_PERSON_ABANDON 2
#define MENU_ALL_ONE_PERSON_EVENTS 3

#define MENU_TWO_PERSON_SEARCH 1
#define MENU_TWO_PERSON_STEAL 2
#define MENU_TWO_PERSON_EXCHANGE 3
#define MENU_ALL_TWO_PERSON_EVENTS 4

#define EVENT_ALL_ONE_PERSON_EVENTS 1
#define EVENT_ALL_TWO_PERSON_EVENTS 2
#define EVENT_ONE_PERSON_SEARCH 3
#define EVENT_ONE_PERSON_ABANDON 4
#define EVENT_TWO_PERSON_SEARCH 5
#define EVENT_TWO_PERSON_STEAL 6
#define EVENT_TWO_PERSON_EXCHANGE 7

//menus
void onMouseEventSelector(int event,int x, int y, int flags, void* data);
void MakePersonColorPicker(cv::Mat* image, cv::Point top_left, const utility::ColorPairListType& colors);
void MakeBagColorPicker(cv::Mat* image, cv::Point top_left, const utility::ColorSingleListType& colors);
void MakeButton(cv::Mat* image, std::string button_text, cv::Point top_left, cv::Scalar color, cv::Point bottom_right);
uint MenuInitial(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors);

uint MenuOnePersonSearch(const utility::ColorPairListType& people_colors);
uint MenuOnePersonAbandon(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors);
uint MenuTwoPersonSearch(const utility::ColorPairListType& people_colors);
uint MenuTwoPersonSteal(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors);
uint MenuTwoPersonExchange(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors);
uint MenuOnePerson(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors);
uint MenuTwoPerson(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors);

uint MenuGetPerson(const utility::ColorPairListType& people_colors);
uint MenuGetBag(const utility::ColorSingleListType& bag_colors);

void onMouseEventSelector(int event,int x, int y, int flags, void* data) {
	if(event==CV_EVENT_LBUTTONDOWN){
		uint* input = (uint*)data;
		*input = y/kMenuItemHeight;
	}
}

void MakeButton(cv::Mat* image, std::string button_text, cv::Point top_left, cv::Scalar color, cv::Point bottom_right=cv::Point()) {
	//set the bottom left if necessary
	if(bottom_right==cv::Point()) {
		bottom_right.x = kMenuWidth;
		bottom_right.y = top_left.y+kMenuItemHeight;
	}
	//draw the color
	cv::rectangle(*image,top_left,bottom_right,color,-1,8);

	//put the text on the window
	cv::putText(*image,button_text,top_left+cv::Point(5,15),CV_FONT_HERSHEY_SIMPLEX,kMenuFontScale,kFontColor,kMenuFontThickness,8);
}

uint DecodeEvent(uint flags) {
	return flags%256;
}
uint DecodeBag(uint flags) {
	flags = flags >> 8;
	return flags%256;
}
uint DecodePersonOne(uint flags) {
	flags = flags >> 16;
	return flags%256;
}
uint DecodePersonTwo(uint flags) {
	flags = flags >> 24;
	return flags%256;
}
uint EncodeEvent(uint flags,uint event) {
	//remove existing flag
	uint existing = flags%256;
	flags -= existing;
	//encode new flag
	flags+=(event%256);
	return flags;
}
uint EncodeBag(uint flags, uint bag) {
	//remove existing flag
	uint existing = ((flags >> 8)%256) << 8;
	flags -= existing;
	//encode new flag
	flags+=(bag%256)<< 8;
	return flags;
}
uint EncodePersonOne(uint flags, uint person_one) {
	//remove existing flag
	uint existing = ((flags >> 16)%256) << 16;
	flags -= existing;
	//encode new flag
	flags+=(person_one%256)<< 16;
	return flags;
}
uint EncodePersonTwo(uint flags, uint person_two) {
	//remove existing flag
	uint existing = ((flags >> 24)%256) << 24;
	flags -= existing;
	//encode new flag
	flags+=(person_two%256)<< 24;
	return flags;
}

uint MenuInitial(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors) {
	cv::Mat menu_image(cv::Size(kMenuWidth,kMenuItemHeight*4),CV_8UC3,cv::Scalar(0,0,0));
	cv::Point top_left(0,0);
	MakeButton(&menu_image,"Please select an event to search for",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"One-person events",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Two-person events",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Search for all events",top_left,cv::Scalar(0,0,0));

	//show the menu
	imshow(kMenuName,menu_image);
	//wait for response
	uint event = NO_EVENT;
	cv::setMouseCallback(kMenuName,onMouseEventSelector,&event);

	while(event==NO_EVENT) if(cv::waitKey(1)=='q') break;
	uint event_code;

	switch(event) {
	case MENU_ONE_PERSON:
		event_code =  MenuOnePerson(bag_colors,people_colors);
		cv::destroyWindow(kMenuName);
		return event_code;
		break;
	case MENU_TWO_PERSON:
		event_code =  MenuTwoPerson(bag_colors,people_colors);
		cv::destroyWindow(kMenuName);
		return event_code;
		break;
	case MENU_ALL_EVENTS:
		event_code = EncodeEvent(event_code,EVENT_ANY);
		event_code = EncodePersonOne(event_code,EVENT_ANY);
		event_code = EncodePersonTwo(event_code,EVENT_ANY);
		event_code = EncodeBag(event_code,EVENT_ANY);

		cv::destroyWindow(kMenuName);
		return event_code;
		break;
	default:
		//the click was outside the menu range
		assert(false);
		break;
	}
	return 0; //this will never happen, it just makes the editor happy

}
uint MenuOnePerson(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors) {
	cv::Mat menu_image(cv::Size(kMenuWidth,kMenuItemHeight*4),CV_8UC3,cv::Scalar(0,0,0));
	cv::Point top_left(0,0);
	MakeButton(&menu_image,"Please select an event to search for",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"One-person search",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"One-person bag abandon",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Search for all one-person events",top_left,cv::Scalar(0,0,0));

	//show the menu
	imshow(kMenuName,menu_image);
	//wait for response
	uint event = NO_EVENT;
	cv::setMouseCallback(kMenuName,onMouseEventSelector,&event);

	while(event==NO_EVENT) if(cv::waitKey(1)=='q') break;
	switch(event) {
	case MENU_ONE_PERSON_SEARCH:
		return MenuOnePersonSearch(people_colors);
		break;
	case MENU_ONE_PERSON_ABANDON:
		return MenuOnePersonAbandon(bag_colors,people_colors);
		break;
	case MENU_ALL_ONE_PERSON_EVENTS: {
		uint event_code=0;
		event_code = EncodeEvent(event_code,EVENT_ALL_ONE_PERSON_EVENTS);
		event_code = EncodePersonOne(event_code,EVENT_ANY);
		event_code = EncodePersonTwo(event_code,EVENT_NOT_USED);
		event_code = EncodeBag(event_code,EVENT_ANY);
		return event_code;
		}
		break;
	default:
		//the click was outside the menu range
		assert(false);
		break;
	}

	return 0; //this will never happen, it just makes the editor happy
}

uint MenuTwoPerson(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors) {
	cv::Mat menu_image(cv::Size(kMenuWidth,kMenuItemHeight*5),CV_8UC3,cv::Scalar(0,0,0));
	cv::Point top_left(0,0);
	MakeButton(&menu_image,"Please select an event to search for",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Two-person search",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Two-person bag steal",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Two-person bag exchange",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Search for all two-person events",top_left,cv::Scalar(0,0,0));

	//show the menu
	imshow(kMenuName,menu_image);
	//wait for response
	uint event = NO_EVENT;
	cv::setMouseCallback(kMenuName,onMouseEventSelector,&event);

	while(event==NO_EVENT) if(cv::waitKey(1)=='q') break;
	switch(event) {
	case MENU_TWO_PERSON_SEARCH:
		return MenuTwoPersonSearch(people_colors);
		break;
	case MENU_TWO_PERSON_STEAL:
		return MenuTwoPersonSteal(bag_colors,people_colors);
		break;
	case MENU_TWO_PERSON_EXCHANGE:
		return MenuTwoPersonExchange(bag_colors,people_colors);
		break;
	case MENU_ALL_TWO_PERSON_EVENTS: {
		uint event_code=0;
		event_code = EncodeEvent(event_code,EVENT_ALL_TWO_PERSON_EVENTS);
		event_code = EncodePersonOne(event_code,EVENT_ANY);
		event_code = EncodePersonTwo(event_code,EVENT_ANY);
		event_code = EncodeBag(event_code,EVENT_ANY);
		return event_code;
		}
		break;
	default:
		//the click was outside the menu range
		assert(false);
		break;
	}

	return 0; //this will never happen, it just makes the editor happy
}

uint MenuOnePersonSearch(const utility::ColorPairListType& people_colors) {
	uint event_code=0;
	event_code = EncodeEvent(event_code,EVENT_ONE_PERSON_SEARCH);
	event_code = EncodePersonOne(event_code,MenuGetPerson(people_colors));
	event_code = EncodePersonTwo(event_code,EVENT_NOT_USED);
	event_code = EncodeBag(event_code,EVENT_NOT_USED);
	return event_code;
}
uint MenuOnePersonAbandon(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors) {
	uint event_code=0;
	event_code = EncodeEvent(event_code,EVENT_ONE_PERSON_ABANDON);
	event_code = EncodePersonOne(event_code,MenuGetPerson(people_colors));
	event_code = EncodePersonTwo(event_code,EVENT_NOT_USED);
	event_code = EncodeBag(event_code,MenuGetBag(bag_colors));
	return event_code;
}
uint MenuTwoPersonSearch(const utility::ColorPairListType& people_colors) {
	uint event_code=0;
	event_code = EncodeEvent(event_code,EVENT_TWO_PERSON_SEARCH);
	event_code = EncodePersonOne(event_code,MenuGetPerson(people_colors));
	event_code = EncodePersonTwo(event_code,MenuGetPerson(people_colors));
	event_code = EncodeBag(event_code,EVENT_NOT_USED);
	return event_code;
}
uint MenuTwoPersonSteal(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors) {
	uint event_code=0;
	event_code = EncodeEvent(event_code,EVENT_TWO_PERSON_STEAL);
	event_code = EncodePersonOne(event_code,MenuGetPerson(people_colors));
	event_code = EncodePersonTwo(event_code,MenuGetPerson(people_colors));
	event_code = EncodeBag(event_code,MenuGetBag(bag_colors));
	return event_code;
}
uint MenuTwoPersonExchange(const utility::ColorSingleListType& bag_colors,const utility::ColorPairListType& people_colors){
	uint event_code=0;
	event_code = EncodeEvent(event_code,EVENT_TWO_PERSON_EXCHANGE);
	event_code = EncodePersonOne(event_code,MenuGetPerson(people_colors));
	event_code = EncodePersonTwo(event_code,MenuGetPerson(people_colors));
	event_code = EncodeBag(event_code,MenuGetBag(bag_colors));
	return event_code;
}







uint MenuGetBag(const utility::ColorSingleListType& bag_colors) {
	if(bag_colors.size()<2) {
		//there's only one option
		return EVENT_ANY;
	}
	uint number_of_options = 2+bag_colors.size();
	cv::Mat menu_image(cv::Size(kMenuWidth,kMenuItemHeight*number_of_options),CV_8UC3,cv::Scalar(0,0,0));
	cv::Point top_left(0,0);
	MakeButton(&menu_image,"Please select a bag's color",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Any known color distribution",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeBagColorPicker(&menu_image,top_left,bag_colors);

	//show the menu
	imshow(kMenuName,menu_image);
	//wait for response
	uint event = NO_EVENT;
	cv::setMouseCallback(kMenuName,onMouseEventSelector,&event);

	while(event==NO_EVENT || event<1) if(cv::waitKey(1)=='q') break;

	if(event==1) {
		//user selected "any known color"
		return EVENT_ANY;
	}
	else {
		//the event number returned should be the person color distribution+1
		if((event-1) > bag_colors.size()) assert(false);
		return event-2;
	}
}

uint MenuGetPerson(const utility::ColorPairListType& people_colors) {
	if(people_colors.size()<2) {
		//there's only one option
		return EVENT_ANY;
	}
	uint number_of_options = 2+people_colors.size();
	cv::Mat menu_image(cv::Size(kMenuWidth,kMenuItemHeight*number_of_options),CV_8UC3,cv::Scalar(0,0,0));
	cv::Point top_left(0,0);
	MakeButton(&menu_image,"Please select a person's color",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakeButton(&menu_image,"Any known color distribution",top_left,cv::Scalar(0,0,0));
	top_left.y+= kMenuItemHeight;
	MakePersonColorPicker(&menu_image,top_left,people_colors);

	//show the menu
	imshow(kMenuName,menu_image);
	//wait for response
	uint event = NO_EVENT;
	cv::setMouseCallback(kMenuName,onMouseEventSelector,&event);

	while(event==NO_EVENT || event<1) if(cv::waitKey(1)=='q') break;

	if(event==1) {
		//user selected "any known color"
		return EVENT_ANY;
	}
	else {
		//the event number returned should be the person color distribution+1
		if((event-1) > people_colors.size()) assert(false);
		return event-2;
	}
}

void MakePersonColorPicker(cv::Mat* image, cv::Point top_left, const utility::ColorPairListType& colors) {
	for(uint color_index=0;color_index<colors.size();color_index++) {
		cv::rectangle(*image,top_left,top_left+cv::Point(kMenuWidth/2,kMenuItemHeight),utility::HSV2BGR(colors[color_index][0]),-1,8);
		cv::rectangle(*image,top_left+cv::Point(kMenuWidth/2,0),top_left+cv::Point(kMenuWidth,kMenuItemHeight),utility::HSV2BGR(colors[color_index][1]),-1,8);
		top_left.y+=kMenuItemHeight;
	}
}

void MakeBagColorPicker(cv::Mat* image, cv::Point top_left, const utility::ColorSingleListType& colors) {
	for(uint color_index=0;color_index<colors.size();color_index++) {
		cv::rectangle(*image,top_left,top_left+cv::Point(kMenuWidth/2,kMenuItemHeight),utility::HSV2BGR(colors[color_index][0]),-1,8);
		top_left.y+=kMenuItemHeight;
	}
}
#endif
