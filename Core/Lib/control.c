/*
 * control.c
 *
 *  Created on: Mar 13, 2026
 *      Author: lazar
 */

#include "main.h"

static void stop();
static void rotate(double dt);
static void go_to_xy(double dt);
static void disassemble();

static void reset_all_pids();

move moves_[MAX_NUM_OF_MOVES];
action ctrl_action_;
uint8_t move_index_ = 0;
uint16_t prev_action_id_ = 0xffff;
double first_x_ = 0.0, first_y_ = 0.0, first_phi_ = 0.0;
int8_t ctrl_move_type_ = -1;

static pose_2D_stamped odom_;
double v_ref_ = 0.0, w_ref_ = 0.0;
double v_ctrl_ = 0.0, w_ctrl_ = 0.0;
double v_ref_ff_ = 0.0, w_ref_ff_ = 0.0;
double v_ref_fb_ = 0.0, w_ref_fb_ = 0.0;

double V_MIN_ = 0.1;
double V_MAX_ = 1.5;
double W_MIN_ = 0.628;
double W_MAX_ = 12.57;
double V_MIN_STACKED_ = 0.01;

double MOTOR_V_MAX_ = 1.6;
double A_MAX_ = 2.0;	// [m/s^2]
double L_ = 0.15;		// TODO: namesti
double STACKED_TIME_ = 0.06;
double D_TOL_ = 0.002; // absolute distance from target
double D_PROJ_TOL_ = 0.0; // projected distance from target
double D_LONG_TOL_ = 0.12; // distance before rotation is used fully
double D_SHORT_TOL_ = 0.03; // minimal distance for rotation during translation
double PHI_TOL_ = 0.0157; // absolute angle from target
double v_right_ref_, v_left_ref_;
double v_right_, v_left_;
double phi_error_, x_error_, y_error_;
unsigned stacked_cnt_ = 0;
double V_SLOWED_MAX_ = 0.75;
volatile pid v_loop, w_loop, d_loop, phi_loop;
double distance_, distance_proj_;        // [m]

void move_init() {
	ctrl_action_.id = 0;

	init_pid(&d_loop, 1.0, 0.0, 0.0, 2.0, 2.0);
	init_pid(&phi_loop, 1.0, 0.0, 0.0, 12.57, 12.57);
	init_pid(&v_loop, 12.0, 0.01, 1.0, 1680, 420);
	init_pid(&w_loop, 92.0, 0.025, 28.0, 1680, 280);
}

void action_init(action *action_ptr) {
	move_index_ = 0;
	first_x_ = get_odom().x;
	first_y_ = get_odom().y;
	first_phi_ = get_odom().phi;
	action_ptr->status = 0;
	for (uint8_t mi = 0; mi < action_ptr->num_of_moves; mi++) {
		move *src = &action_ptr->moves_ptr[mi];
		move *prev = NULL;
		double move_phi = 0.0, prev_phi = 0.0;
		double move_distance = 0.0, x_e = 0.0, y_e = 0.0;
		if (mi > 0)
			prev = &action_ptr->moves_ptr[mi - 1];
		src->index = mi;
		src->status = 0;

		switch (src->type) {
		case -1:
//			stop();
			src->a = fmin(src->wheel_accel, A_MAX_);
			src->alpha = 2.0 * src->a / L_;
			src->v_max = V_MAX_;
			src->w_max = W_MAX_;
			break;
		case 0:
//			disassemble();
			src->a = 0.0;
			src->alpha = 0.0;
			src->v_max = 0.0;
			src->w_max = 0.0;
			break;
		case 1:
//			go_to_xy(dt);
			if (prev) {
				x_e = src->x - prev->x;
				y_e = src->y - prev->y;
				src->phi = prev->phi;
			} else {
				x_e = src->x - first_x_;
				y_e = src->y - first_y_;
				src->phi = first_phi_;
			}
			move_distance = sqrt(x_e * x_e + y_e * y_e);
			if (prev)
				prev_phi = prev->phi;
			else
				prev_phi = first_phi_;
			move_phi = wrap(src->phi - prev_phi, -M_PI, M_PI);

			src->a = fmin(src->wheel_accel, A_MAX_);
			src->alpha = 2.0 * src->a / L_;
			src->v_max = compute_v_peak(fmin(src->v_des, V_MAX_), src->a,
					move_distance);
			src->w_max = compute_v_peak(fmin(src->w_des, W_MAX_), src->alpha,
					fabs(move_phi));
			break;
		case 2:
			// TODO: kriva
			src->a = fmin(src->wheel_accel, A_MAX_);
			src->alpha = 2.0 * src->a / L_;
//	TODO:	izracunaj
//			move_distance
			src->v_max = compute_v_peak(fmin(src->v_des, V_MAX_), src->a,
					move_distance);
			src->w_max = W_MAX_;
			break;
		case 3:
//			rotate(dt);
			if (prev) {
				prev_phi = prev->phi;
				src->x = prev->x;
				src->y = prev->y;
			} else {
				prev_phi = first_phi_;
				src->x = first_x_;
				src->y = first_y_;
			}
			move_phi = wrap(src->phi - prev_phi, -M_PI, M_PI);

			src->a = fmin(src->wheel_accel, A_MAX_);
			src->alpha = 2.0 * src->a / L_;
			src->v_max = V_MAX_;
			src->w_max = compute_v_peak(fmin(src->w_des, W_MAX_), src->alpha,
					fabs(move_phi));
			break;
		}
		memcpy(&moves_[mi], src, sizeof(move));
	}
}

void control_loop(double dt) {
	pwm_init();
	odom_ = get_odom();
	if (prev_action_id_ != ctrl_action_.id)
		action_init(&ctrl_action_);

	if (move_index_ < ctrl_action_.num_of_moves) {
		if (moves_[move_index_].status < 0)
			move_index_++;
	}
	if (move_index_ >= ctrl_action_.num_of_moves) {
		ctrl_action_.status = -1;
		ctrl_move_type_ = -1;
	} else {
		ctrl_action_.status = move_index_ + 1;
		ctrl_move_type_ = moves_[move_index_].type;
	}

	switch (ctrl_move_type_) {
	case -1:
		stop();
		break;
	case 0:
		disassemble();
		break;
	case 1:
		go_to_xy(dt);
		break;
	case 2:
		// TODO: kriva
		break;
	case 3:
		rotate(dt);
		break;
	}
	v_ref_ = v_ref_ff_ + v_ref_fb_;
	w_ref_ = w_ref_ff_ + w_ref_fb_;
	prev_action_id_ = ctrl_action_.id;
}

void velocity_loop(double dt) {
	double v_err = v_ref_ - get_odom().v;
	double w_err = w_ref_ - get_odom().w;
	v_ctrl_ = calc_pid(&v_loop, v_err, dt);
	w_ctrl_ = calc_pid(&w_loop, w_err, dt);

	v_right_ = vel_ramp(v_right_, v_ctrl_ + w_ctrl_ * L_ * 0.5, A_MAX_ * dt);
	v_left_ = vel_ramp(v_left_, v_ctrl_ - w_ctrl_ * L_ * 0.5, A_MAX_ * dt);
	scale_vel_ref(&v_right_, &v_left_, MOTOR_V_MAX_);

	pwm_right(v_right_, MOTOR_V_MAX_);
	pwm_left(v_left_, MOTOR_V_MAX_);
}

static void rotate(double dt) {
	switch (moves_[move_index_].status) {
	case 0:
		reset_all_pids();
		moves_[move_index_].status = 1;
		// namerno izbacen break
	case 1:
		phi_error_ = wrap(moves_[move_index_].phi - odom_.phi, -M_PI, M_PI);
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_) {
			moves_[move_index_].status = -1;
			reset_all_pids();
			break;
		}
		v_ref_ff_ = 0;
		w_ref_ff_ = trajectory_synthesis(moves_[move_index_].w_max, w_ref_ff_,
				moves_[move_index_].alpha, phi_error_, dt);
		break;
	}
}

static void go_to_xy(double dt) {
	x_error_ = moves_[move_index_].x - odom_.x;
	y_error_ = moves_[move_index_].y - odom_.y;
	phi_error_ = wrap(
			atan2(y_error_, x_error_) - odom_.phi
					+ (moves_[move_index_].direction - 1) * M_PI * 0.5, -M_PI,
			M_PI);
	switch (moves_[move_index_].status) {
	case 0:
		reset_all_pids();
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_) {
			moves_[move_index_].status = 3;
			break;
		} else {
			moves_[move_index_].status = 1;
		}
	case 1:
		if (fabs(phi_error_) < PHI_TOL_ && fabs(odom_.w) < W_MIN_) {
			moves_[move_index_].status = 2;
			break;
		}
		v_ref_ff_ = 0;
		w_ref_ff_ = trajectory_synthesis(moves_[move_index_].w_max, w_ref_ff_,
				moves_[move_index_].alpha, phi_error_, dt);
		break;
	case 2:
		reset_all_pids();
		moves_[move_index_].status = 3;
		// namerno izbacen break
	case 3:
		distance_ = sqrt(x_error_ * x_error_ + y_error_ * y_error_);
		distance_proj_ = distance_ * cos(phi_error_);

		if (distance_proj_ < D_PROJ_TOL_ && fabs(odom_.v) < V_MIN_
				&& fabs(odom_.w) < W_MIN_) {
			reset_all_pids();
			if (fabs(distance_) < D_TOL_) {
				moves_[move_index_].status = -1;
				break;
			} else {
				moves_[move_index_].status = -5;
				break;
			}
		} else if (stacked(STACKED_TIME_, odom_.v, V_MIN_STACKED_, 1.0 / dt,
				&stacked_cnt_)) {
			reset_all_pids();
			moves_[move_index_].status = -3;
			break;
		}

		v_ref_ff_ = trajectory_synthesis(moves_[move_index_].v_max, v_ref_ff_,
				moves_[move_index_].a, distance_, dt);
		w_ref_ff_ = 0.0;
		double phi_err_mod_ = clamp(
				(distance_ - D_SHORT_TOL_) / (D_LONG_TOL_ - D_SHORT_TOL_), 0.0,
				1.0) * phi_error_;
		w_ref_fb_ = calc_pid(&phi_loop, phi_err_mod_, dt);
		break;
	}
}

// TODO: proveri za status
static void stop() {
	if (move_index_ < ctrl_action_.num_of_moves)
		moves_[move_index_].status = -1;
	v_ref_ = 0.0;
	w_ref_ = 0.0;
	stacked_cnt_ = 0;
	reset_all_pids();
}

static void disassemble() {
	stop();
	pwm_kill();
	if (move_index_ < ctrl_action_.num_of_moves)
		moves_[move_index_].status = -1;
}

static void reset_all_pids() {
	reset_pid(&d_loop);
	reset_pid(&phi_loop);
	reset_pid(&v_loop);
	reset_pid(&w_loop);
	v_ref_ff_ = 0.0;
	w_ref_ff_ = 0.0;
	v_ref_fb_ = 0.0;
	w_ref_fb_ = 0.0;
	stacked_cnt_ = 0;
}

double get_v_r() {
	return v_right_;
}

double get_v_l() {
	return v_left_;
}
