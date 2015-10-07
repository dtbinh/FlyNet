/*
 * trackerMain.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: taylordean
 */

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "trackerMain.hpp"
#include "OBJECT.hpp"
#include "TRACK.hpp"
#include "TRACKLIST.hpp"
#include "OBJECT_CONTAINER.hpp"
#include "amm.hpp"

using namespace cv;
using namespace std;

void morphImage(Mat imgIn, Mat * imgOut, int iLowH, int iLowS,
        int iLowV, int iHighH, int iHighS, int iHighV)
{
    Mat imgHSV;
    cvtColor(imgIn, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), *imgOut); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(*imgOut, *imgOut, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(*imgOut, *imgOut, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(*imgOut, *imgOut, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(*imgOut, *imgOut, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
}

void getContours(OBJECT_CONTAINER* objects, Mat imgContour)
{
	vector< vector < Point > > contours;
	vector< vector < Point > > finalContours;
	vector< double > areas;
	time_t gmt = time(0);

	findContours(imgContour, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (int contourIdx=0; contourIdx<contours.size(); contourIdx++) {
		double area = contourArea(contours[contourIdx], false);
		if (area > MIN_OBJECT_AREA) {
			areas.push_back(area);
			finalContours.push_back(contours[contourIdx]);
		}
	}

	for (int retIdx=0; retIdx<MAX_OBJECTS; retIdx++) {
		if (finalContours.empty()) {
			break;
		} else {
			int maxIdx = 0;
			for (int contourIdx=0; contourIdx<finalContours.size(); contourIdx++) {
				if (areas[maxIdx] < areas[contourIdx]) {
					maxIdx = contourIdx;
				}
			}

			Point centroid;
			findCentroid(finalContours[maxIdx], &centroid);
			Rect brect = boundingRect(Mat(finalContours[maxIdx]).reshape(2));

			OBJECT newObject(objects->size(), -1, gmt, areas[maxIdx], centroid, brect);
			objects->append(newObject);

			finalContours.erase(finalContours.begin()+maxIdx);
			areas.erase(areas.begin()+maxIdx);


		}
	}
}

void findCentroid(vector< Point > points, Point * centroid)
{
	        float sumx = 0;
	        float sumy = 0;
			for (int i=0; i<points.size(); i++) {
				sumx = sumx + points[i].x;
				sumy = sumy + points[i].y;
			}
			int posX = sumx / points.size();
			int posY = sumy / points.size();

			*centroid = Point(posX, posY);
}

int main( int argc, char** argv )
{
	OBJECT_CONTAINER GLOBAL_RETURNS;
	TRACKLIST GLOBAL_TRACKS;

    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() ) {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", WINDOW_NORMAL); //create a window called "Control"

    int iLowH = 39;
	int iHighH = 115;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 50;
	int iHighV = 255;

    //Create trackbars in "Control" window
    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);

    //Capture a temporary image from the camera
    Mat imgTmp;
    cap.read(imgTmp);

    //Create a black image with the size as the camera output
    Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;

    while (true) {
        Mat imgOriginal;
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        if (!bSuccess) {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        Mat imgThresholded;
        morphImage(imgOriginal, &imgThresholded, iLowH, iLowS, iLowV,
                iHighH, iHighS, iHighV);

        getContours(&GLOBAL_RETURNS, imgThresholded.clone());

        assembleTracks(&GLOBAL_RETURNS, &GLOBAL_TRACKS);

        for (int trkIdx=0; trkIdx<GLOBAL_TRACKS.size(); trkIdx++) {
			if (GLOBAL_TRACKS[trkIdx].size() > 0) {
				rectangle(imgOriginal, GLOBAL_TRACKS[trkIdx].newest()->brect.tl(), GLOBAL_TRACKS[trkIdx].newest()->brect.br(), Scalar(100, 100, 200), 2, CV_AA);
				rectangle(imgThresholded, GLOBAL_TRACKS[trkIdx].newest()->brect.tl(), GLOBAL_TRACKS[trkIdx].newest()->brect.br(), Scalar(100, 100, 200), 2, CV_AA);
				line(imgLines, GLOBAL_TRACKS[trkIdx].newest()->centroid, GLOBAL_TRACKS[trkIdx][GLOBAL_TRACKS[trkIdx].size() - 2]->centroid, Scalar(0,0,255), 2);
			}
        }

        flip(imgThresholded, imgThresholded, 1);
        resize(imgThresholded, imgThresholded, Size(), 0.5, 0.5, CV_INTER_AREA);
        imshow("Thresholded Image", imgThresholded); //show the thresholded image

        imgOriginal = imgOriginal + imgLines;
        flip(imgOriginal, imgOriginal, 1);
        resize(imgOriginal, imgOriginal, Size(), 0.5, 0.5, CV_INTER_AREA);
        imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

   return 0;
}
