/*
 * trackerMain.h
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#ifndef TRACKERMAIN_HPP_
#define TRACKERMAIN_HPP_

#include "TRACK.hpp"
#include "OBJECT_CONTAINER.hpp"

#define MAX_OBJECTS 1
#define MIN_OBJECT_AREA 1000

using namespace cv;
using namespace std;

void morphImage(Mat imgIn, Mat * imgOut, int iLowH, int iLowS,
        int iLowV, int iHighH, int iHighS, int iHighV);
void getContours(OBJECT_CONTAINER* objects, Mat imgContour);
void findCentroid(vector< Point > points, Point * centroid);



#endif /* TRACKERMAIN_HPP_ */
