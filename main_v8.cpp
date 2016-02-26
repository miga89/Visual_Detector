/*
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

bool recordVideo = true;
int main(int argc, char* argv[])
{

	
	//Matrix to store each frame of the webcam feed, Mat is an opencv c++ n-dimensional array class
	Mat cameraFeed, frame, croppedFrame;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;

	// create state history container
	double state_x = 0;
	double state_y = 0;
	double state_vx = 0;
	double state_vy = 0;
	double x_prev = 0;
	double y_prev = 0;
	Point_<double> ballpx(0, 0); //create px-location of ball
	Point_<double> ballcd(0, 0); //create coordinate-location of ball
	Point_<double> ballcd_cor(0, 0); //create Kalman corrected coordinate-location of ball
	Point_<double> ballpx_cor(0, 0); //create Kalman corrected pixel-location of ball
	cv::Rect predRect(0, 0, 250, 250);

	// debugging variables
	int dbg = 1;
	//cv::Mat dspPrediction = state; // variable used to print predictions on screen

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
	double t =0;
	// create text file to store data and delete previous data
	std::ofstream data;
	std::ofstream pdata;
	int data_counter = 0;
	data.open("data.txt");
	data << "RECONSTRUCT CALCULATIONS \n";
	data.close();
		data.open("data.txt", std::ios_base::app);
		 data <<  intToString(mycase) + " " + dblToString(t) + " " + dblToString(ballcd.x) + " " + dblToString(ballcd.y) + "\n";
		data.close();

		// Setup output video
		int fcc = CV_FOURCC('M','P','4','2');
		cv::VideoWriter output_cap("D:/CamCapture.avi",fcc,20, cvSize(  1920, 1200 ) );
	
	// test function


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

			// predict next state
			makePrediction(dT,state_x,state_y,state_vx,state_vy);
			data.open("data.txt", std::ios_base::app);
			 data <<  "makePrediction():\n";
			 data <<  "dT: " + dblToString(dT) + " state_x:" + dblToString(state_x) + " state_y:" + dblToString(state_y)+ " state_vx:" + dblToString(state_vx)+ " state_vy:" + dblToString(state_vy) + " \n \n";
			data.close();
			putText(cameraFeed, "xp=" + dblToString(state_x) + " yp=" + dblToString(state_y) + " vxp=" + dblToString(state_vx) + " vyp=" + dblToString(state_vy), Point(0, 200), 1, 2, Scalar(255, 0, 0), 2);

			//dspPrediction = state; // variable used to print predictions on screen
			// save predicted states to file to evaluate kalman filter
			//pdata.open("predict.txt", std::ios_base::app);
			//pdata <<  intToString(mycase) + " " +  dblToString(state.at<float>(0)) + " " +  dblToString(state.at<float>(1))+ " " +  dblToString(state.at<float>(2))+ " " +  dblToString(state.at<float>(3))+ "\n";
			//pdata.close();
			//state = kf.predict();


			//calculate center point
			ballcd.x = state_x; 
			ballcd.y = state_y;
			coordinatesToPx(ballpx.x, ballpx.y, ballcd.x, ballcd.y);
			//std::cout << "xpx:" << ballpx.x << "; ypx:" << ballpx.y << "; xcd:" << ballcd.x << "; ycd:" << ballcd.y << "\n"; 
			// draw circle into the resulting picture, centered around predicted location, and radius 2
			cv::circle(cameraFeed, ballpx, 10, CV_RGB(0, 255, 0), 2);
			cv::circle(cameraFeed, ballpx, 3, CV_RGB(0, 255, 0), -1);

			// create and intialize prediction rectangle
			predRect.x = ballpx.x - (predRect.width / 2); // calculate the top left corner x 
			predRect.y = ballpx.y - (predRect.height / 2); // calculate the top left corner y
			// adjust predRect to make sure it is within picture
			adjustPredRect(predRect);
			// draw rectangle into the resulting picture  with the information specified in predRect
			cv::rectangle(cameraFeed, predRect, CV_RGB(0, 255, 0), 2);
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
			// after calculating the ball's px location, calculate coordinates
			pxToCoordinates(ballpx.x, ballpx.y, ballcd.x, ballcd.y); 
			// draw information to screen
			cv::circle(cameraFeed, ballpx, 3, CV_RGB(0, 0, 255), -1);
			putText(cameraFeed, "Object Found at: x=" + dblToString(ballcd.x) + "mm (" + dblToString(ballpx.x) + "px) , y=" + dblToString(ballcd.y) + "mm (" + dblToString(ballpx.y) + "px)", Point(0, 50), 2, 1, Scalar(255, 0, 0), 2);

			numAttempt = 0; //set numAttempt to 0
			
			
	
			// calculate new state
			x_prev = state_x;
			y_prev = state_y;
			state_vx = (ballcd.x - x_prev)/dT;
			state_vy = (ballcd.y - y_prev)/dT;
			if (state_vx > 2) state_vx = 2;
			if (state_vx < -2) state_vx = -2;
			if (state_vy > 2) state_vy = 2;
			if (state_vy < -2) state_vy = -2;
			state_x = ballcd.x;
			state_y = ballcd.y;

			data.open("data.txt", std::ios_base::app);
			data <<  "Object detected!: \n";
			data <<  "ballcd.x:" + dblToString(ballcd.x) + " ballcd.y:" + dblToString(ballcd.y)+ "\n";
			data <<  "x_prev:" + dblToString(x_prev) + " y_prev:" + dblToString(y_prev) + " dT:" + dblToString(dT)+ "\n";
			data <<  "state_x:" + dblToString(state_x) + " state_y:" + dblToString(state_y)  + " state_vx:" + dblToString(state_vx) + " state_vy:" + dblToString(state_vy)+ "\n \n";
			data.close();


			//draw object location on screen, input centroid position and current camera frame
			//drawObject(x_px, y_px, cameraFeed);

		}
		else
		{
			numAttempt++;
			putText(cameraFeed, "No Object Found!", Point(0, 50), 1, 2, Scalar(255, 0, 0), 2);
					data.open("data.txt", std::ios_base::app);
			data <<  "No Object detected!: \n";
			data <<  "ballcd.x:" + dblToString(ballcd.x) + " ballcd.y:" + dblToString(ballcd.y)+ "\n";
			data <<  "x_prev:" + dblToString(x_prev) + " y_prev:" + dblToString(y_prev) + " dT:" + dblToString(dT)+ "\n";
			data <<  "state_x:" + dblToString(state_x) + " state_y:" + dblToString(state_y)  + " state_vx:" + dblToString(state_vx) + " state_vy:" + dblToString(state_vy)+ "\n \n";
			data.close();
		}


		// Add information to camera feed
		//putText(cameraFeed, "+", Point(x_px_est, y_px_est), 1, 2, Scalar(0, 0, 255), 2);
		line(cameraFeed, vertical_left_top, vertical_left_bottom, Scalar(255, 255, 255), 2, 8);
		line(cameraFeed, vertical_right_top, vertical_right_bottom, Scalar(255, 255, 255), 2, 8);
		line(cameraFeed, horizontal_top_left, horizontal_top_right, Scalar(255, 255, 255), 2, 8);
		line(cameraFeed,horizontal_bottom_left, horizontal_bottom_right, Scalar(255, 255, 255), 2, 8);
			putText(cameraFeed, "case=" + intToString(mycase)+ " dT= " + dblToString(dT) , Point(0, 350), 1, 2, Scalar(255, 0, 0), 2);
		
		if (counter == 10){
			stop = (double)cv::getTickCount();
			measuredFPS =10/((stop-start)/ cv::getTickFrequency());
			counter = 0;
		}
		
		
		putText(cameraFeed, "FPS=" + dblToString(measuredFPS), Point(0, 100), 1, 2, Scalar(255, 0, 0), 2);

		//imshow(thresholdedImage, threshold);
		imshow(originalImage, cameraFeed);
		imshow(hsvImage, HSV);

		// save data to text file
		timestop = (double)cv::getTickCount();
		t = (timestop - timestart) / cv::getTickFrequency();

	


		output_cap.write(cameraFeed);
		if (t>20) break;
	
		
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(1);
	}


	if (recordVideo == true){
		capture.release();
		output_cap.release();
	}


	return EXIT_SUCCESS;
}
*/