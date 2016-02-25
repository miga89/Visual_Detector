#define _CRT_SECURE_NO_DEPRECATE

#ifndef FUNCTIONS_H
#define FUNCTIONS_H


double computeFPS(clock_t clock1, clock_t clock2);

double computeTime(clock_t clock1, clock_t clock2);

inline const char * const BoolToString(bool b);

void pxToCoordinates(double x_px, double y_px, double &x_cd, double &y_cd);

void coordinatesToPx(double &x_px, double &y_px, double x_cd, double y_cd);

void adjustPredRect(cv::Rect &predRect);

void makePrediction(double dT, double &state_x, double &state_y, double &state_vx,double & state_vy);


#endif