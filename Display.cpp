#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv\highgui.h>
#include <opencv\cv.h>
#include "Display.h"
#include "ImageAquisition.h"
#include "Converter.h"

using namespace std;
using namespace cv;

//initial min and max HSV filter values.
//these will be changed using trackbars
// HSV colorspace used to filter colors
//halogen licht an 


// mit dem autocolor bestimmt (beide lichter an)
int H_MIN = 15;
int H_MAX = 20;
int S_MIN = 212;
int S_MAX = 255;
int V_MIN = 186;
int V_MAX = 230;

// mit dem autocolor bestimmt (deckenlicht aus)
//int H_MIN = 13;
//int H_MAX = 17;
//int S_MIN = 255;
//int S_MAX = 255;
//int V_MIN = 149;
//int V_MAX = 189;
//names that will appear at the top of each window
const std::string originalImage = "Original Image";
const std::string hsvImage = "HSV Image";
const std::string thresholdedImage = "Thresholded Image";
const std::string croppedImage = "Cropped Image";
const std::string trackbarWindowName = "Trackbars";

// create valus for the white horizontal and vertical lines displayed on screen that are used for camera calibration
cv::Point_<double> vertical_left_top(120, 0);
cv::Point_<double> vertical_left_bottom(120, FRAME_HEIGHT);

cv::Point_<double> vertical_right_top(FRAME_WIDTH - 325, 0);
cv::Point_<double> vertical_right_bottom(FRAME_WIDTH - 325, FRAME_HEIGHT);

cv::Point_<double> horizontal_top_left(0, 170);
cv::Point_<double> horizontal_top_right(FRAME_WIDTH, 170);

cv::Point_<double> horizontal_bottom_left(0, FRAME_HEIGHT - 440);
cv::Point_<double> horizontal_bottom_right(FRAME_WIDTH, FRAME_HEIGHT - 440);
double line_delta_x = abs(vertical_right_top.x - vertical_left_bottom.x);
double line_delta_y = abs(horizontal_bottom_left.y - horizontal_top_left.y);

void on_trackbar(int, void*)
{//This function gets called whenever a
	// trackbar position is changed
	// ich glaube nur eine dummy funktion, wenn die trackbar geschoben wird

}

void createTrackbars(){
	//create window for trackbars

	// namedWindow(Window Name,int flag for Options (WINDOW_NORMAL, WINDOW_AUTOSIZE, WINDOW_OPENGL))
	cv::namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	cv::createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
	cv::createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
	cv::createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
	cv::createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
	cv::createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
	cv::createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}

void drawObject(int x, int y, cv::Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, cv::Point(x, y), 20, cv::Scalar(255, 0, 0), 2);
	if (y - 25>0)
		line(frame, cv::Point(x, y), cv::Point(x, y - 25), cv::Scalar(255, 0, 0), 2);
	else line(frame, cv::Point(x, y), cv::Point(x, 0), cv::Scalar(255, 0, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, cv::Point(x, y), cv::Point(x, y + 25), cv::Scalar(255, 0, 0), 2);
	else line(frame, cv::Point(x, y), cv::Point(x, FRAME_HEIGHT), cv::Scalar(255, 0, 0), 2);
	if (x - 25>0)
		line(frame, cv::Point(x, y), cv::Point(x - 25, y), cv::Scalar(255, 0, 0), 2);
	else line(frame, cv::Point(x, y), cv::Point(0, y), cv::Scalar(255, 0, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, cv::Point(x, y), cv::Point(x + 25, y), cv::Scalar(255, 0, 0), 2);
	else line(frame, cv::Point(x, y), cv::Point(FRAME_WIDTH, y), cv::Scalar(255, 0, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), cv::Point(x, y + 30), 1, 1, cv::Scalar(255, 0, 0), 2);

}

void createWindows()
{
	//start an infinite loop where camera feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop

	namedWindow(originalImage, WINDOW_NORMAL);
	namedWindow(thresholdedImage, WINDOW_NORMAL);
	namedWindow(hsvImage, WINDOW_NORMAL);
	namedWindow(croppedImage, WINDOW_NORMAL);

	// move and resize windows to place them on the screen in a convenient manner
	moveWindow(originalImage, 0, 0);
	moveWindow(thresholdedImage, 840, 0);
	moveWindow(hsvImage, 0, 525);
	moveWindow(trackbarWindowName, 1200, 525);
	//namedWindow(windowName4,WINDOW_NORMAL);
}
