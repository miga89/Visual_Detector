#ifndef DISPLAY_H
#define DISPLAY_H

#include <string>
extern int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;
extern const std::string originalImage, hsvImage, thresholdedImage, croppedImage, trackbarWindowName;
extern cv::Point_<double> vertical_left_top;
extern cv::Point_<double> vertical_left_bottom;

extern cv::Point_<double> vertical_right_top;
extern cv::Point_<double> vertical_right_bottom;

extern cv::Point_<double> horizontal_top_left;
extern cv::Point_<double> horizontal_top_right;

extern cv::Point_<double> horizontal_bottom_left;
extern cv::Point_<double> horizontal_bottom_right;
extern double line_delta_x;
extern double line_delta_y;

extern cv::Point_<double> horizontal_bottom_right;



// declare functions
void on_trackbar(int, void*);
void drawObject(int x, int y, cv::Mat &frame);
void createTrackbars();
void createWindows();
#endif
