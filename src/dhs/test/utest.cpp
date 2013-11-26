/*
 * utest.cpp
    Copyright (C) 2013  Timothy Sweet

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

// Bring in my package's API, which is what I'm testing
#include "../include/Utility.h"
// Bring in gtest
#include <gtest/gtest.h>

TEST(Utility,easyBox)
{
	cv::Mat image(cv::Size(5,5),CV_8UC1,cv::Scalar(0));
	cv::rectangle(image,cv::Point(1,1),cv::Point(3,3),cv::Scalar(255),-1);
	int median = utility::getHorizontalMedian(image);

	EXPECT_EQ(median,2);
}
TEST(Utility,edgeCaseSmallWhite)
{
	cv::Mat image(cv::Size(1,1),CV_8UC1,cv::Scalar(0));
	cv::rectangle(image,cv::Point(1,1),cv::Point(1,1),cv::Scalar(255),-1);
	int median = utility::getHorizontalMedian(image);

	EXPECT_EQ(median,0);
}
TEST(Utility,edgeCaseSmallBlack)
{
	cv::Mat image(cv::Size(1,1),CV_8UC1,cv::Scalar(0));
	int median = utility::getHorizontalMedian(image);

	EXPECT_EQ(median,0);
}
TEST(Utility,twoBoxes)
{
	cv::Mat image(cv::Size(50,50),CV_8UC1,cv::Scalar(0));
	cv::rectangle(image,cv::Point(5,5),cv::Point(9,9),cv::Scalar(255),-1);
	cv::rectangle(image,cv::Point(15,15),cv::Point(19,19),cv::Scalar(255),-1);

	int median = utility::getHorizontalMedian(image);

	EXPECT_EQ(median,12);
}


int main(int argc, char* argv[]) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
