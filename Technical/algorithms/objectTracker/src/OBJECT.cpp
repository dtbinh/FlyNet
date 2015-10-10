/*
 * OBJECT.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#include "OBJECT.hpp"

using namespace cv;

OBJECT::OBJECT(void)
{
	retId = -1;
	trkId = -1;
	area = -1;
	gmt = -1;
	centroid = Point(-1, -1);
	brect = Rect(Point(-1, -1), Point(-1, -1));

}

OBJECT::OBJECT(int retIdIn, int trkIdIn, time_t gmtIn, float areaIn, Point centroidIn, Rect brectIn)
{
	retId = retIdIn;
	trkId = trkIdIn;
	area = areaIn;
	gmt = gmtIn;
	centroid = centroidIn;
	brect = brectIn;
}
