#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv\highgui.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "ImageAquisition.h"
#include "DataOutput.h"
#include "Converter.h"
#include "Display.h"
#include "ImageProcessing.h"
#include "KalmanFilter.h"
#include "Functions.h"
double computeFPS(clock_t clock1, clock_t clock2)
{
	double diffticks = clock1 - clock2;
	double diffs = (diffticks) / CLOCKS_PER_SEC;
	double fps = 10 / diffs;
	return fps;
}

double computeTime(clock_t clock1, clock_t clock2)
{
	double diffticks = clock1 - clock2;
	double diffs = (diffticks) / CLOCKS_PER_SEC;
	return diffs;
}

inline const char * const BoolToString(bool b)
{
	return b ? "true" : "false";
}

void pxToCoordinates(double x_px, double y_px, double &x_cd, double &y_cd)
{
	if (x_px == -1 && y_px == -1)
	{
		x_cd = -1;
		y_cd = -1;
	}
	else
	{
		//x_cd = x_px * 3330 / 1570;
		//y_cd = (FRAME_HEIGHT - y_px) * 1300 / 675 + 600 - 400 * 1300 / 675;
		x_cd = x_px * 3330.0 / 1505.0;
		y_cd = (FRAME_HEIGHT - y_px) * 1300.0 / 595.0;
		//x_cd = x_px * (3300.0 /line_delta_x);
		//y_cd = (FRAME_HEIGHT - y_px) * 1300.0 / (line_delta_y);

	}
}

void coordinatesToPx(double &x_px, double &y_px, double x_cd, double y_cd)
{
	if (x_cd == -1 && y_cd == -1)
	{
		x_px = -1;
		y_px = -1;
	}
	else
	{
		//x_px = x_cd * 1570.0 / 3300.0;
		//y_px = (675.0 / 1300.0) * ((FRAME_HEIGHT * 1300.0 / 675.0) - y_cd + 600.0 - (400.0 * 1300.0 / 675));
		x_px = x_cd * 1505.0 / 3300.0;
		y_px = (595.0 / 1300.0) * ((FRAME_HEIGHT * 1300.0 / 595.0) - y_cd);
		//x_px = x_cd * (line_delta_x / 3300.0);
		//y_px = FRAME_HEIGHT - (y_cd * (line_delta_y/1300.0));

	}
}

void adjustPredRect(cv::Rect &predRect)
{
	float x = predRect.x;
	float y = predRect.y;
	float w = predRect.width;
	float h = predRect.height;

	if (x < 0) x = 0;
	else if (x > FRAME_WIDTH - w) x = FRAME_WIDTH - w;

	if (y < 0) y = 0;
	else if (y > FRAME_HEIGHT - h) y = FRAME_HEIGHT - h;

	predRect.x = x;
	predRect.y = y;

}