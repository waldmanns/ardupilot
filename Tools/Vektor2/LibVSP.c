/*
 * swio_libvsp.c
 *
 *  Created on: May 4, 2021
 *      Author: Samson Waldmann
 */


#include "LibVSP.h"

#include <math.h>

struct Limiter setupLimiter(int32_t rad, int32_t x_off, int32_t y_off) {
	struct Limiter out;
	out.radius = rad;
	out.x_offset = x_off;
	out.y_offset = y_off;
	return out;
}

struct Coordinates mapCoordinates(struct Coordinates input, struct Limiter limits, int32_t angle_correction) {
	//declaring output
	struct Coordinates output;
	//declaring auxiliary variables
	double center_x = (double)limits.x_offset;
	double center_y = (double)limits.y_offset;

	double radius = (double)limits.radius;

	double ist_x = (double)input.x_coordinate;
	double ist_y = (double)input.y_coordinate;

	double soll_x = (double)input.x_coordinate;  //copying value of actual coordinate, just in case the calculation fails..
	double soll_y = (double)input.y_coordinate;  //idem

	double m; //line angle
	double d; //distance from center


	//preventing some x/0 and 0/0 situations
	if(ist_y == center_y && ist_x == center_x) {
		return input;
	}

	double theta = angle_correction * (3.14159265358979323846 / 180.0);

	m = atan((ist_y - center_y) / (ist_x - center_x))*-1;

	m = m+theta;

	d = sqrt(pow((ist_x - center_x),2.0)+pow((ist_y - center_y),2.0));

	if(d <= radius) {
		if(ist_x >= center_x && ist_y >= center_y) {
			soll_x = center_x + d*cos(m);
			soll_y = center_y + d*sin(m)*-1;
		}
		else if(ist_x >= center_x && ist_y < center_y) {
			soll_x = center_x + d*cos(m);
			soll_y = center_y + d*sin(m)*-1;
		}
		else if(ist_x < center_x && ist_y >= center_y) {
			soll_x = center_x + d*cos(m)*-1;
			soll_y = center_y + d*sin(m);
		}
		else if(ist_x < center_x && ist_y < center_y) {
			soll_x = center_x + d*cos(m)*-1;
			soll_y = center_y + d*sin(m);
		}
		else {
					return input;
		}
		output.x_coordinate = (int32_t)soll_x;
		output.y_coordinate = (int32_t)soll_y;
	}
	//else
	else {
		if(ist_x >= center_x && ist_y >= center_y) {
			soll_x = center_x + radius*cos(m);
			soll_y = center_y + radius*sin(m)*-1;
		}
		else if(ist_x >= center_x && ist_y < center_y) {
			soll_x = center_x + radius*cos(m);
			soll_y = center_y + radius*sin(m)*-1;
		}
		else if(ist_x < center_x && ist_y >= center_y) {
			soll_x = center_x + radius*cos(m)*-1;
			soll_y = center_y + radius*sin(m);
		}
		else if(ist_x < center_x && ist_y < center_y) {
			soll_x = center_x + radius*cos(m)*-1;
			soll_y = center_y + radius*sin(m);
		}
		else {
			return input;
		}
		output.x_coordinate = (int32_t)soll_x;
		output.y_coordinate = (int32_t)soll_y;


	}

	return output;
}
