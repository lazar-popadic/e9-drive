/*
 * odometry.c
 *
 *  Created on: Dec 23, 2025
 *      Author: lazar
 */

#include "main.h"

double L_odom, L_odom_recip, d_odom_left, d_odom_right;

double mid_angle;
volatile double v_right, v_left;
volatile double dis_right, dis_left;
volatile pose_2D_stamped odom;

void odometry_init() {
	L_odom = 0.2993;			// [m], rastojanje izmedju odometrijskih tockova
	L_odom_recip = 1.0 / L_odom;	// [1/m]
	d_odom_left = 0.07509;			// [m]
	d_odom_right = 0.07528;			// [m]
}

void update_odom(double dt) {
	v_right = enc_velocity(&htim5, dt, 8192) * d_odom_right;		// [m/s]
	v_left = enc_velocity(&htim3, dt, 8192) * d_odom_left;

	dis_right += v_right;
	dis_left += v_left;

	odom.v = (v_right + v_left) * 0.5;				// [m/s]
	odom.w = (v_right - v_left) * L_odom_recip;		// [rad/s]
	mid_angle = (odom.phi + odom.w * 0.5 * dt);		// [rad]

	odom.x += odom.v * cos(mid_angle) * dt;			// [m]
	odom.y += odom.v * sin(mid_angle) * dt;
	odom.phi += odom.w * dt;						// [rad]

	wrap2Pi(&odom.phi);
	odom.time_ms = get_time();
}

pose_2D_stamped get_odom() {
	return odom;
}
