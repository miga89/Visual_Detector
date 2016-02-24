#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "KalmanFilter.h"

cv::KalmanFilter createKalmanFilter(int stateSize, int measSize, int contrSize, unsigned int type)
{
	
	// FORMULAS:
	// xdot = A*x + B*u + w with Cov(w)=Q
	// y = C*x + v with Cov(v) = R

	// create kalman filter object kf
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	
	//A 
	cv::setIdentity(kf.transitionMatrix); //initialize state matrix A with and identity
	kf.transitionMatrix.at<float>(10) = 1; //CHANGE LATER
	kf.transitionMatrix.at<float>(15) = 1;

	// B
	kf.controlMatrix = cv::Mat::zeros(stateSize, 1,type);

	// C
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type); //initialize C matrix as a zero matrix
	kf.measurementMatrix.at<float>(0) = 1.0f; //C matrix; add a float value of 1.0f to the four entries in the matrix
	kf.measurementMatrix.at<float>(5) = 1.0f;
	
	
	// R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1)); //GEÄNDERT!!!!!!!!! R 1e-1

	// Q
	kf.processNoiseCov.at<float>(0) = 1e-2; //Q Matrix
	kf.processNoiseCov.at<float>(5) = 1e-2;
	kf.processNoiseCov.at<float>(10) = 5.0f;
	kf.processNoiseCov.at<float>(15) = 5.0f;

	// P, initialization
	kf.errorCovPre.at<float>(0) = 1; // px
	kf.errorCovPre.at<float>(5) = 1; // px
	kf.errorCovPre.at<float>(10) = 1;
	kf.errorCovPre.at<float>(15) = 1;

	return kf;
}

cv::Mat kalmanPredict(cv::KalmanFilter &kf, cv::Mat control) //prediction = kf.predict(control);
{
	cv::Mat prediction(6, 1, CV_32F);
	prediction = kf.predict(control);
	//prediction = kf.predict();
	kf.statePre.copyTo(kf.statePost);
	kf.errorCovPre.copyTo(kf.errorCovPost);
	return prediction;
}


