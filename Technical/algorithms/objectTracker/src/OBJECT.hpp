/*
 * OBJECT.hpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#ifndef OBJECT_HPP_
#define OBJECT_HPP_

#include <ctime>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class OBJECT
{
public:
	// Parameters
	int retId;
	int trkId;
	time_t gmt;
	float area;
	Point centroid;
	Rect brect;

	// Functions
	OBJECT(void);
	OBJECT(int retIdIn, int trkIdIn, time_t gmtIn, float areaIn, Point centroidIn, Rect brectIn);
};

#endif /* OBJECT_HPP_ */
