
// Michael Garstka
// Ping Pong Ball tracker
// Version of 15.02.16
#define _CRT_SECURE_NO_DEPRECATE

#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv\highgui.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <conio.h> //to use key pressing commands

#include "ImageAquisition.h"
#include "DataOutput.h"
#include "Converter.h"
#include "Display.h"
#include "ImageProcessing.h"
#include "KalmanFilter.h"
#include "Functions.h"

using namespace cv;

int main(int argc, char* argv[])
{

	
	//Matrix to store each frame of the webcam feed, Mat is an opencv c++ n-dimensional array class
	Mat cameraFeed, frame, croppedFrame;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;

	// create kalman Filter object and state and measurement vectors
	unsigned int type = CV_32F;
	int stateSize = 4; // [x,y,v_x,v_y]
	int measSize = 2; // [z_x,z_y]
	int contrSize = 1;
	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
	cv::Rect predRect(0, 0, 250, 250);
	cv::Point center(0, 0);
	cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	state.at<float>(0) = 0;
	state.at<float>(1) = 0;
	state.at<float>(2) = 0;
	state.at<float>(3) = 0;
	cv::Mat control(contrSize, 1, type);
	control.at<float>(0) = 1000;
	cv::KalmanFilter kf = createKalmanFilter(stateSize, measSize, contrSize, type);
	cv::Mat correct = meas;
	// create Points for ball position in px and coordinates
	Point_<double> ballpx(0, 0); //create px-location of ball
	Point_<double> ballcd(0, 0); //create coordinate-location of ball
	Point_<double> dspPredPos(0, 0); //needed for displaying predicted position

	// debugging variables
	int dbg = 1;
	cv::Mat dspPrediction(stateSize,1,type);

	//create slider bars for HSV filtering
	//createTrackbars();

	//video capture object to acquire webcam feed is created with object name "capture"
	cv::VideoCapture capture = createVideoCapture(FRAME_WIDTH, FRAME_HEIGHT);
	
	//create windows to display picture and results
	createWindows();

	// define counter for loop counts
	int counter = 0;
	double measuredFPS = 0;
	bool objectDetected = false;
	int numAttempt = 100;
	bool initial = true;
	bool searchSection = false;
	int mycase = 0;

	// define variables for time measurements
	//clock_t start, stop, stampstart, stampstop;
	double start, stop;
	double timestart = (double)cv::getTickCount();
	double ticks = 0;
	double dT = 0;
	double precTick = 0;
	double timestop = 0;
	double t=0;
	
	// create text file to store data and delete previous data
	std::ofstream data;
	std::ofstream pdata;
	int datanum = 100; // int appended to filename


	// Setup videowriter object
	int fcc = CV_FOURCC('M','P','4','2');
	cv::VideoWriter output_cap("D:/CamCapture.avi",fcc,20, cvSize(  1920, 1200 ) );
	

	// MAIN LOOP
	while (1){

		if (counter == 0)
		{
			start = (double)cv::getTickCount();
		}
		counter += 1;

		// compute time dT
		precTick = ticks; //save previous "time measurement"
		ticks = (double)cv::getTickCount(); //get new "time measurement"

		dT = (ticks - precTick) / cv::getTickFrequency(); //compute dT

		//capture.read(Mat& image) grabs,decodes and returns the nexxt video frame and stores image to matrix camerafeed
		capture.read(cameraFeed);

		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// CASE CHECKER: check if we want to search the whole image or just a section of it

		// Case 1: Initial loop run --> search whole picture
		if (initial == true)
		{


			searchSection = false;
			initial = false;
			mycase = 1;
		}
		else
		{
			// Case 2: Object Detected in previous run --> search only around next estimated position
			if (objectDetected == true)
			{
				searchSection = true; 
				mycase = 2;
			}
			// Case 3: No object detected in previous run AND object was detected within last 5 runs
			else if (objectDetected == false && numAttempt <= 15)
			{
				searchSection = true; 
				mycase = 3;
			}
			// Case 4: No object detected in previous run AND no object was detected within last 5 runs
			else if (objectDetected == false && numAttempt > 15)
			{
				searchSection = false;
				mycase = 4;
			}
		}
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //

		frame = cameraFeed;
		if (searchSection == true)
		{
			// update transition matrix A
			kf.transitionMatrix.at<float>(2) = dT;
			kf.transitionMatrix.at<float>(7) = dT;
			// udpate control matrix B
			kf.controlMatrix.at<float>(3) = -dT;

			// predict next state
			state = kalmanPredict(kf,control); //slightly modified
			dspPrediction = state; // variable used to print predictions on screen

			//calculate center point
			ballcd.x = state.at<float>(0); 
			ballcd.y = state.at<float>(1);
			coordinatesToPx(ballpx.x, ballpx.y, ballcd.x, ballcd.y); //calculate px from coordinates
			dspPredPos = ballpx;

			// create and intialize prediction rectangle
			predRect.x = ballpx.x - (predRect.width / 2); // calculate the top left corner x 
			predRect.y = ballpx.y - (predRect.height / 2); // calculate the top left corner y
			// adjust predRect to make sure it is within picture
			adjustPredRect(predRect);

			// crop the image
			cameraFeed(predRect).copyTo(croppedFrame);
			frame = croppedFrame;

		}

		// perform image operations on image
		cvtColor(frame, HSV, COLOR_BGR2HSV);
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
		morphOps(threshold);
		//imwrite( "feed.jpg", cameraFeed );
		//imwrite( "cvt.jpg", HSV );
		//imwrite( "thresh.jpg", threshold );
		// try to track the object in the cropped threshold image and return pixel coordinates of object, otherwise return -1,-1
		trackFilteredObject(ballpx.x, ballpx.y, threshold, frame, objectDetected, dbg);
		if (objectDetected == true)
		{
			if (searchSection == true)
			{
				// recalculate global pixel coordinates from cropped pixel coordinates
				if (ballpx.x != -1 || ballpx.y != -1)
				{
					ballpx.x = predRect.x + ballpx.x;
					ballpx.y = predRect.y + ballpx.y;
				}
			}
			// after finding the ball's px location, calculate coordinates
			pxToCoordinates(ballpx.x, ballpx.y, ballcd.x, ballcd.y); 
			
			// draw information to screen
			cv::circle(cameraFeed, ballpx, 3, CV_RGB(0, 0, 255), -1);
			putText(cameraFeed, "Object Found at: x=" + dblToString(ballcd.x) + "mm (" + dblToString(ballpx.x) + "px) , y=" + dblToString(ballcd.y) + "mm (" + dblToString(ballpx.y) + "px)", Point(0, 50), 2, 1, Scalar(255, 0, 0), 2);

			numAttempt = 0; //set numAttempt to 0


			// Kalman Correct
			meas.at<float>(0) = ballcd.x; //update the measurement vector with the x value
			meas.at<float>(1) = ballcd.y; //update the measurement vector with y value
			correct = kf.correct(meas); 


			

		}
		else
		{
			numAttempt++;
			putText(cameraFeed, "No Object Found!", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
			correct = kf.statePost; // save Kalman Correction=Kalman Prediction

		}

		if (mycase == 2 || mycase ==3){
			// draw circle into the resulting picture, centered around predicted location, and radius 2
			cv::circle(cameraFeed, dspPredPos, 10, CV_RGB(0, 255, 0), 2);
			cv::circle(cameraFeed, dspPredPos, 3, CV_RGB(0, 255, 0), -1);
			// draw prediction rectangle on screen
			cv::rectangle(cameraFeed, predRect, CV_RGB(0, 255, 0), 2);
		}
	
		// Add information to camera feed
		// calibration lines
		line(cameraFeed, vertical_left_top, vertical_left_bottom, Scalar(255, 255, 255), 2, 8);
		line(cameraFeed, vertical_right_top, vertical_right_bottom, Scalar(255, 255, 255), 2, 8);
		line(cameraFeed, horizontal_top_left, horizontal_top_right, Scalar(255, 255, 255), 2, 8);
		line(cameraFeed,horizontal_bottom_left, horizontal_bottom_right, Scalar(255, 255, 255), 2, 8);
		// Additional debugging info
		putText(cameraFeed, "xp=" + dblToString(dspPrediction.at<float>(0)) + " yp=" + dblToString(dspPrediction.at<float>(1)) + " vxp=" + dblToString(dspPrediction.at<float>(2)) + " vyp=" + dblToString(dspPrediction.at<float>(3)), Point(0, 200), 1, 2, Scalar(255, 0, 0), 2);
		putText(cameraFeed, "xc=" + dblToString(correct.at<float>(0)) + " yc=" + dblToString(correct.at<float>(1)) + " vxc=" + dblToString(correct.at<float>(2)) + " vyc=" + dblToString(correct.at<float>(3)), Point(0, 300), 1, 2, Scalar(255, 0, 0), 2);
		putText(cameraFeed, "case=" + intToString(mycase) , Point(0, 350), 1, 2, Scalar(255, 0, 0), 2);
		
		// used to measure FPS
		if (counter == 10){
			stop = (double)cv::getTickCount();
			measuredFPS =10/((stop-start)/ cv::getTickFrequency());
			counter = 0;
		}
		// display computed FPS
		putText(cameraFeed, "FPS=" + dblToString(measuredFPS), Point(0, 100), 1, 2, Scalar(255, 0, 0), 2);

		// decide which images to show
		//imshow(thresholdedImage, threshold);
		imshow(originalImage, cameraFeed);
		//imshow(hsvImage, HSV);

		// save data to text file
		timestop = (double)cv::getTickCount();
		t = (timestop - timestart) / cv::getTickFrequency();
		data.open("data"+intToString(datanum)+".txt", std::ios_base::app);
		data <<  intToString(mycase) + " " + dblToString(t) + " " + dblToString(ballcd.x) + " " + dblToString(ballcd.y)+ " "+ dblToString(ballpx.x) + " " + dblToString(ballpx.y)+"\n";
		data.close();
	

		// Uncomment if you want to capture a video for 20s
		//	output_cap.write(cameraFeed);
		//if (t>20) break;
	
		
		//image will not appear without this waitKey() command
		waitKey(1);
			


	}

		// in case of video capturing, release the objects
		capture.release();
		output_cap.release();



	return EXIT_SUCCESS;
}
