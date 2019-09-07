
#include "zhp_control_app.h"


ctrl_val_typedef ctrl_val;
auto_ctrl_val_typedef auto_ctrl_val;
uint16_t motor_a_pwm;
uint16_t motor_b_pwm;

float pwm_resolution;
double velocity, angular;
void zhp_control_init()
{
	auto_ctrl_val.p_t = -30000;
	auto_ctrl_val.i_t = -3000;
	auto_ctrl_val.d_t = -5000;
	
	auto_ctrl_val.accel_cmd = 1.0;
	auto_ctrl_val.max_accel = 1.0;
	auto_ctrl_val.min_accel = 0.1;
	auto_ctrl_val.max_speed = 0.1;
	auto_ctrl_val.min_speed = 0.0;
	auto_ctrl_val.max_angular = 1.0;
	auto_ctrl_val.min_angular = 0.0;
	
	auto_ctrl_val.spd_pid_lower_left.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_lower_left.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_lower_left.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_lower_left.ek = 0;
	auto_ctrl_val.spd_pid_lower_left.ek1 = 0;
	auto_ctrl_val.spd_pid_lower_left.target = 0;
	auto_ctrl_val.spd_pid_lower_left.pout = 0;
	auto_ctrl_val.spd_pid_lower_left.iout = 0;
	auto_ctrl_val.spd_pid_lower_left.isum = 0;
	auto_ctrl_val.spd_pid_lower_left.dout = 0;
	auto_ctrl_val.spd_pid_lower_left.out =0;
	auto_ctrl_val.spd_pid_lower_left.cur = 0;

	auto_ctrl_val.spd_pid_higher_left.p = -auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_higher_left.i = -auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_higher_left.d = -auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_higher_left.ek = 0;
	auto_ctrl_val.spd_pid_higher_left.ek1 = 0;
	auto_ctrl_val.spd_pid_higher_left.target = 0;
	auto_ctrl_val.spd_pid_higher_left.pout = 0;
	auto_ctrl_val.spd_pid_higher_left.iout = 0;
	auto_ctrl_val.spd_pid_higher_left.isum = 0;
	auto_ctrl_val.spd_pid_higher_left.dout = 0;
	auto_ctrl_val.spd_pid_higher_left.out =0;
	auto_ctrl_val.spd_pid_higher_left.cur = 0;

	auto_ctrl_val.spd_pid_lower_right.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_lower_right.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_lower_right.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_lower_right.ek = 0;
	auto_ctrl_val.spd_pid_lower_right.ek1 = 0;
	auto_ctrl_val.spd_pid_lower_right.target = 0;
	auto_ctrl_val.spd_pid_lower_right.pout = 0;
	auto_ctrl_val.spd_pid_lower_right.iout = 0;
	auto_ctrl_val.spd_pid_lower_right.isum = 0;
	auto_ctrl_val.spd_pid_lower_right.dout = 0;
	auto_ctrl_val.spd_pid_lower_right.out =0;
	auto_ctrl_val.spd_pid_lower_right.cur = 0;

	auto_ctrl_val.spd_pid_higher_right.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_higher_right.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_higher_right.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_higher_right.ek = 0;
	auto_ctrl_val.spd_pid_higher_right.ek1 = 0;
	auto_ctrl_val.spd_pid_higher_right.target = 0;
	auto_ctrl_val.spd_pid_higher_right.pout = 0;
	auto_ctrl_val.spd_pid_higher_right.iout = 0;
	auto_ctrl_val.spd_pid_higher_right.isum = 0;
	auto_ctrl_val.spd_pid_higher_right.dout = 0;
	auto_ctrl_val.spd_pid_higher_right.out =0;
	auto_ctrl_val.spd_pid_higher_right.cur = 0;
}

void clear_i()
{
	auto_ctrl_val.spd_pid_lower_left.iout = 0;
	auto_ctrl_val.spd_pid_lower_left.isum = 0;
	
	auto_ctrl_val.spd_pid_higher_left.iout = 0;
	auto_ctrl_val.spd_pid_higher_left.isum = 0;
	
	auto_ctrl_val.spd_pid_lower_right.iout = 0;
	auto_ctrl_val.spd_pid_lower_right.isum = 0;
	
	auto_ctrl_val.spd_pid_higher_right.iout = 0;
	auto_ctrl_val.spd_pid_higher_right.isum = 0;
}

void clear_pid()
{
	auto_ctrl_val.spd_pid_lower_left.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_lower_left.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_lower_left.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_lower_left.ek = 0;
	auto_ctrl_val.spd_pid_lower_left.ek1 = 0;
	auto_ctrl_val.spd_pid_lower_left.target = 0;
	auto_ctrl_val.spd_pid_lower_left.pout = 0;
	auto_ctrl_val.spd_pid_lower_left.iout = 0;
	auto_ctrl_val.spd_pid_lower_left.isum = 0;
	auto_ctrl_val.spd_pid_lower_left.dout = 0;
	auto_ctrl_val.spd_pid_lower_left.out =0;
	auto_ctrl_val.spd_pid_lower_left.cur = 0;

	auto_ctrl_val.spd_pid_higher_left.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_higher_left.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_higher_left.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_higher_left.ek = 0;
	auto_ctrl_val.spd_pid_higher_left.ek1 = 0;
	auto_ctrl_val.spd_pid_higher_left.target = 0;
	auto_ctrl_val.spd_pid_higher_left.pout = 0;
	auto_ctrl_val.spd_pid_higher_left.iout = 0;
	auto_ctrl_val.spd_pid_higher_left.isum = 0;
	auto_ctrl_val.spd_pid_higher_left.dout = 0;
	auto_ctrl_val.spd_pid_higher_left.out =0;
	auto_ctrl_val.spd_pid_higher_left.cur = 0;

	auto_ctrl_val.spd_pid_lower_right.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_lower_right.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_lower_right.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_lower_right.ek = 0;
	auto_ctrl_val.spd_pid_lower_right.ek1 = 0;
	auto_ctrl_val.spd_pid_lower_right.target = 0;
	auto_ctrl_val.spd_pid_lower_right.pout = 0;
	auto_ctrl_val.spd_pid_lower_right.iout = 0;
	auto_ctrl_val.spd_pid_lower_right.isum = 0;
	auto_ctrl_val.spd_pid_lower_right.dout = 0;
	auto_ctrl_val.spd_pid_lower_right.out =0;
	auto_ctrl_val.spd_pid_lower_right.cur = 0;

	auto_ctrl_val.spd_pid_higher_right.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_higher_right.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_higher_right.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_higher_right.ek = 0;
	auto_ctrl_val.spd_pid_higher_right.ek1 = 0;
	auto_ctrl_val.spd_pid_higher_right.target = 0;
	auto_ctrl_val.spd_pid_higher_right.pout = 0;
	auto_ctrl_val.spd_pid_higher_right.iout = 0;
	auto_ctrl_val.spd_pid_higher_right.isum = 0;
	auto_ctrl_val.spd_pid_higher_right.dout = 0;
	auto_ctrl_val.spd_pid_higher_right.out =0;
	auto_ctrl_val.spd_pid_higher_right.cur = 0;
}

void update_pid_param()
{	
	//if (auto_ctrl_val.spd)
	auto_ctrl_val.spd_pid_lower_left.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_lower_left.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_lower_left.d = auto_ctrl_val.d_t;

	auto_ctrl_val.spd_pid_higher_left.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_higher_left.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_higher_left.d = auto_ctrl_val.d_t;

	auto_ctrl_val.spd_pid_lower_right.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_lower_right.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_lower_right.d = auto_ctrl_val.d_t;

	auto_ctrl_val.spd_pid_higher_right.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_higher_right.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_higher_right.d = auto_ctrl_val.d_t;
}

void pid_calc(pid_t* pid)
{
	pid->ek = pid->target - pid->cur;
	pid->pout = pid->ek * pid->p;
	double temp = pid->isum + pid->ek * pid->i; 
	if ( temp <= MotoPwmMax || temp >= -MotoPwmMax)
	{
		pid->isum = temp;
	}
	pid->iout = pid->isum;
	pid->dout = (pid->ek - pid->ek1) * pid->d;
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->ek1 = pid->ek;
}

/*********************************************
 *功能：通过PID算法控制小车电机
 *********************************************/
void pid_speed_control()
{
	if (auto_ctrl_val.velocity_cmd == 0.0 && auto_ctrl_val.angular_cmd == 0.0 && auto_ctrl_val.break_state == 1)
	{
		auto_ctrl_val.lower_left_pwm = 0;
		auto_ctrl_val.higher_left_pwm = 0;
		auto_ctrl_val.lower_right_pwm = 0;
		auto_ctrl_val.higher_right_pwm = 0;
	}
	else
	{
		pid_calc(&auto_ctrl_val.spd_pid_lower_left);
		pid_calc(&auto_ctrl_val.spd_pid_higher_left);
		pid_calc(&auto_ctrl_val.spd_pid_lower_right);
		pid_calc(&auto_ctrl_val.spd_pid_higher_right);
		
		auto_ctrl_val.lower_left_pwm = auto_ctrl_val.spd_pid_lower_left.out;
		auto_ctrl_val.higher_left_pwm = auto_ctrl_val.spd_pid_higher_left.out;
		auto_ctrl_val.lower_right_pwm = auto_ctrl_val.spd_pid_lower_right.out;
		auto_ctrl_val.higher_right_pwm = auto_ctrl_val.spd_pid_higher_right.out;
	}
	
	moto_control_with_val();
}

//void check_pwm_limit(int16_t *pwm)
//{
//	if (pwm > )
//}

void check_limit(float *speed, float *angular, float *accel)
{
	if (*speed > auto_ctrl_val.max_speed)
	{
		*speed = auto_ctrl_val.max_speed;
	}
	else if (*speed < -auto_ctrl_val.max_speed)
	{
		*speed = -auto_ctrl_val.max_speed;
	}
	
	if (*angular > auto_ctrl_val.max_angular)
	{
		*angular = auto_ctrl_val.max_angular;
	}
	else if (*angular < -auto_ctrl_val.max_angular)
	{
		*angular = -auto_ctrl_val.max_angular;
	}
	
	if (*accel > auto_ctrl_val.max_accel)
	{
		*accel = auto_ctrl_val.max_accel;
	}
	else if (*accel < -auto_ctrl_val.max_accel)
	{
		*accel = -auto_ctrl_val.max_accel;
	}
}

void arrange_transition_process(float *velocity_cmd, float velocity_cur)
{
//	float accel_piece = auto_ctrl_val.max_accel / 10;
//	
//	if ((*velocity_cmd > 0 && velocity_cur > 0) ||
//	    (*velocity_cmd < 0 && velocity_cur < 0))
//	{
//		return;
//	}
//	
//	if (fabs(*velocity_cmd - velocity_cur) > accel_piece)
//	{
//		if (*velocity_cmd - velocity_cur > 0)
//		{
//			*velocity_cmd = velocity_cur + accel_piece;
//		}
//		else
//		{
//			*velocity_cmd = velocity_cur - accel_piece;
//		}
//	}
}

//#define MENUAL_DEBUG_MODE
float speed;
void auto_ctrl_val_update()
{
	float constant = perimeter / auto_ctrl_val.period;
	
	// calc speed
	auto_ctrl_val.lower_left_speed = auto_ctrl_val.lower_left_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.lower_left_cnt = 0.0;
	auto_ctrl_val.higher_left_speed = auto_ctrl_val.higher_left_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.higher_left_cnt = 0.0;
	auto_ctrl_val.lower_right_speed = auto_ctrl_val.lower_right_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.lower_right_cnt = 0.0;
	auto_ctrl_val.higher_right_speed = auto_ctrl_val.higher_right_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.higher_right_cnt = 0.0;
	
	speed = auto_ctrl_val.lower_right_speed;
	
	float l_speed = (auto_ctrl_val.lower_left_speed + auto_ctrl_val.higher_left_speed) / 2.0;
	float r_speed = (auto_ctrl_val.lower_right_speed + auto_ctrl_val.higher_right_speed) / 2.0;
	
	auto_ctrl_val.velocity_feedback = (l_speed + r_speed) / 2.0;
	auto_ctrl_val.angular_feedback = (r_speed - l_speed) / two_wheel_dist;
	
	frame_data.velocity_feedback = auto_ctrl_val.velocity_feedback;
	frame_data.angular_feedback = auto_ctrl_val.angular_feedback;
	
	#ifndef MENUAL_DEBUG_MODE
	// calc target speed
	// get target speed
	static float last_angular;
	last_angular = auto_ctrl_val.angular_cmd;
  sscanf((char *)final_rx_frame, "%f %f %d", &auto_ctrl_val.velocity_cmd, &auto_ctrl_val.angular_cmd, &auto_ctrl_val.break_state);
	if (last_angular == 0 && auto_ctrl_val.angular_cmd != 0)
	{
		clear_i();
	}
	#endif
	
	check_limit(&auto_ctrl_val.velocity_cmd, &auto_ctrl_val.angular_cmd, &auto_ctrl_val.accel_cmd);
	
	float temp = (two_wheel_dist * auto_ctrl_val.angular_cmd) / 2.0;
	auto_ctrl_val.lower_left_target_speed = auto_ctrl_val.velocity_cmd - temp;
	auto_ctrl_val.higher_left_target_speed = auto_ctrl_val.velocity_cmd - temp;
	auto_ctrl_val.lower_right_target_speed = auto_ctrl_val.velocity_cmd + temp;
	auto_ctrl_val.higher_right_target_speed = auto_ctrl_val.velocity_cmd + temp;
	
	auto_ctrl_val.spd_pid_lower_left.cur = auto_ctrl_val.lower_left_speed;
	auto_ctrl_val.spd_pid_higher_left.cur = auto_ctrl_val.higher_left_speed;
	auto_ctrl_val.spd_pid_lower_right.cur = auto_ctrl_val.lower_right_speed;
	auto_ctrl_val.spd_pid_higher_right.cur = auto_ctrl_val.higher_right_speed;
	
//	arrange_transition_process(&auto_ctrl_val.lower_left_target_speed, auto_ctrl_val.spd_pid_lower_left.cur);
//	arrange_transition_process(&auto_ctrl_val.higher_left_target_speed, auto_ctrl_val.spd_pid_higher_left.cur);
//	arrange_transition_process(&auto_ctrl_val.lower_right_target_speed, auto_ctrl_val.spd_pid_lower_right.cur);
//	arrange_transition_process(&auto_ctrl_val.higher_right_target_speed, auto_ctrl_val.spd_pid_higher_right.cur);
	
	auto_ctrl_val.spd_pid_lower_left.target = auto_ctrl_val.lower_left_target_speed;
	auto_ctrl_val.spd_pid_higher_left.target = auto_ctrl_val.higher_left_target_speed;
	auto_ctrl_val.spd_pid_lower_right.target = auto_ctrl_val.lower_right_target_speed;
	auto_ctrl_val.spd_pid_higher_right.target = auto_ctrl_val.higher_right_target_speed;
	
//	update_pid_param();
	pid_speed_control();

}

/*********************************************
 *功能：电脑控制小车
 *********************************************/
void zhp_computer_control()
{                      
  auto_ctrl_val_update();
}
