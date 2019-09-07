
#include "zhp_control_app.h"


ctrl_val_typedef ctrl_val;
auto_ctrl_val_typedef auto_ctrl_val;
uint16_t motor_a_pwm;
uint16_t motor_b_pwm;

float pwm_resolution;
double velocity, angular;
void zhp_control_init()
{
	auto_ctrl_val.p_t = 1000;
	auto_ctrl_val.i_t = 30;
	auto_ctrl_val.d_t = -2400;
	
	auto_ctrl_val.accel_cmd = 1.0;
	auto_ctrl_val.max_accel = 6.0;
	auto_ctrl_val.min_accel = 0.1;
	auto_ctrl_val.max_speed = 1.0;
	auto_ctrl_val.min_speed = 0.15;
	auto_ctrl_val.max_angular = 3.14;
	auto_ctrl_val.min_angular = 0.2;
	
	auto_ctrl_val.spd_pid_l0.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_l0.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_l0.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_l0.ek = 0;
	auto_ctrl_val.spd_pid_l0.ek1 = 0;
	auto_ctrl_val.spd_pid_l0.target = 0;
	auto_ctrl_val.spd_pid_l0.pout = 0;
	auto_ctrl_val.spd_pid_l0.iout = 0;
	auto_ctrl_val.spd_pid_l0.isum = 0;
	auto_ctrl_val.spd_pid_l0.dout = 0;
	auto_ctrl_val.spd_pid_l0.out =0;
	auto_ctrl_val.spd_pid_l0.cur = 0;

	auto_ctrl_val.spd_pid_l1.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_l1.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_l1.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_l1.ek = 0;
	auto_ctrl_val.spd_pid_l1.ek1 = 0;
	auto_ctrl_val.spd_pid_l1.target = 0;
	auto_ctrl_val.spd_pid_l1.pout = 0;
	auto_ctrl_val.spd_pid_l1.iout = 0;
	auto_ctrl_val.spd_pid_l1.isum = 0;
	auto_ctrl_val.spd_pid_l1.dout = 0;
	auto_ctrl_val.spd_pid_l1.out =0;
	auto_ctrl_val.spd_pid_l1.cur = 0;

	auto_ctrl_val.spd_pid_r0.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_r0.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_r0.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_r0.ek = 0;
	auto_ctrl_val.spd_pid_r0.ek1 = 0;
	auto_ctrl_val.spd_pid_r0.target = 0;
	auto_ctrl_val.spd_pid_r0.pout = 0;
	auto_ctrl_val.spd_pid_r0.iout = 0;
	auto_ctrl_val.spd_pid_r0.isum = 0;
	auto_ctrl_val.spd_pid_r0.dout = 0;
	auto_ctrl_val.spd_pid_r0.out =0;
	auto_ctrl_val.spd_pid_r0.cur = 0;

	auto_ctrl_val.spd_pid_r1.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_r1.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_r1.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_r1.ek = 0;
	auto_ctrl_val.spd_pid_r1.ek1 = 0;
	auto_ctrl_val.spd_pid_r1.target = 0;
	auto_ctrl_val.spd_pid_r1.pout = 0;
	auto_ctrl_val.spd_pid_r1.iout = 0;
	auto_ctrl_val.spd_pid_r1.isum = 0;
	auto_ctrl_val.spd_pid_r1.dout = 0;
	auto_ctrl_val.spd_pid_r1.out =0;
	auto_ctrl_val.spd_pid_r1.cur = 0;
}

void clear_i()
{
	auto_ctrl_val.spd_pid_l0.iout = 0;
	auto_ctrl_val.spd_pid_l0.isum = 0;
	
	auto_ctrl_val.spd_pid_l1.iout = 0;
	auto_ctrl_val.spd_pid_l1.isum = 0;
	
	auto_ctrl_val.spd_pid_r0.iout = 0;
	auto_ctrl_val.spd_pid_r0.isum = 0;
	
	auto_ctrl_val.spd_pid_r1.iout = 0;
	auto_ctrl_val.spd_pid_r1.isum = 0;
}

void clear_pid()
{
	auto_ctrl_val.spd_pid_l0.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_l0.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_l0.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_l0.ek = 0;
	auto_ctrl_val.spd_pid_l0.ek1 = 0;
	auto_ctrl_val.spd_pid_l0.target = 0;
	auto_ctrl_val.spd_pid_l0.pout = 0;
	auto_ctrl_val.spd_pid_l0.iout = 0;
	auto_ctrl_val.spd_pid_l0.isum = 0;
	auto_ctrl_val.spd_pid_l0.dout = 0;
	auto_ctrl_val.spd_pid_l0.out =0;
	auto_ctrl_val.spd_pid_l0.cur = 0;

	auto_ctrl_val.spd_pid_l1.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_l1.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_l1.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_l1.ek = 0;
	auto_ctrl_val.spd_pid_l1.ek1 = 0;
	auto_ctrl_val.spd_pid_l1.target = 0;
	auto_ctrl_val.spd_pid_l1.pout = 0;
	auto_ctrl_val.spd_pid_l1.iout = 0;
	auto_ctrl_val.spd_pid_l1.isum = 0;
	auto_ctrl_val.spd_pid_l1.dout = 0;
	auto_ctrl_val.spd_pid_l1.out =0;
	auto_ctrl_val.spd_pid_l1.cur = 0;

	auto_ctrl_val.spd_pid_r0.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_r0.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_r0.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_r0.ek = 0;
	auto_ctrl_val.spd_pid_r0.ek1 = 0;
	auto_ctrl_val.spd_pid_r0.target = 0;
	auto_ctrl_val.spd_pid_r0.pout = 0;
	auto_ctrl_val.spd_pid_r0.iout = 0;
	auto_ctrl_val.spd_pid_r0.isum = 0;
	auto_ctrl_val.spd_pid_r0.dout = 0;
	auto_ctrl_val.spd_pid_r0.out =0;
	auto_ctrl_val.spd_pid_r0.cur = 0;

	auto_ctrl_val.spd_pid_r1.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_r1.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_r1.d = auto_ctrl_val.d_t;
	auto_ctrl_val.spd_pid_r1.ek = 0;
	auto_ctrl_val.spd_pid_r1.ek1 = 0;
	auto_ctrl_val.spd_pid_r1.target = 0;
	auto_ctrl_val.spd_pid_r1.pout = 0;
	auto_ctrl_val.spd_pid_r1.iout = 0;
	auto_ctrl_val.spd_pid_r1.isum = 0;
	auto_ctrl_val.spd_pid_r1.dout = 0;
	auto_ctrl_val.spd_pid_r1.out =0;
	auto_ctrl_val.spd_pid_r1.cur = 0;
}

void update_pid_param()
{	
	//if (auto_ctrl_val.spd)
	auto_ctrl_val.spd_pid_l0.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_l0.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_l0.d = auto_ctrl_val.d_t;

	auto_ctrl_val.spd_pid_l1.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_l1.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_l1.d = auto_ctrl_val.d_t;

	auto_ctrl_val.spd_pid_r0.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_r0.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_r0.d = auto_ctrl_val.d_t;

	auto_ctrl_val.spd_pid_r1.p = auto_ctrl_val.p_t;
	auto_ctrl_val.spd_pid_r1.i = auto_ctrl_val.i_t;
	auto_ctrl_val.spd_pid_r1.d = auto_ctrl_val.d_t;
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
	pid->iout = pid->isum * pid->i;
	pid->dout = (pid->ek - pid->ek1) * pid->d;
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->ek1 = pid->ek;
}

/*********************************************
 *功能：通过PID算法控制小车电机
 *********************************************/
void pid_speed_control(float velocity, float angular)
{
	if (velocity > auto_ctrl_val.min_speed || velocity < -auto_ctrl_val.min_speed ||
		  angular > auto_ctrl_val.min_angular || angular < -auto_ctrl_val.min_angular)
	{
		pid_calc(&auto_ctrl_val.spd_pid_l0);
		pid_calc(&auto_ctrl_val.spd_pid_l1);
		pid_calc(&auto_ctrl_val.spd_pid_r0);
		pid_calc(&auto_ctrl_val.spd_pid_r1);
		
		auto_ctrl_val.l0_pwm = auto_ctrl_val.spd_pid_l0.out;
		auto_ctrl_val.l1_pwm = auto_ctrl_val.spd_pid_l1.out;
		auto_ctrl_val.r0_pwm = auto_ctrl_val.spd_pid_r0.out;
		auto_ctrl_val.r1_pwm = auto_ctrl_val.spd_pid_r1.out;
		
		moto_control_with_val();
	}
	else
	{
		clear_pid();
		
		MOTOR_A0_SetBreak(MotoPwmMax);
		MOTOR_A1_SetBreak(MotoPwmMax);
		MOTOR_B0_SetBreak(MotoPwmMax);
		MOTOR_B1_SetBreak(MotoPwmMax);
	}
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

void auto_ctrl_val_update()
{
	float constant = perimeter / auto_ctrl_val.period;
	
	// calc speed
	auto_ctrl_val.l0_speed = auto_ctrl_val.l0_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.l0_cnt = 0.0;
	auto_ctrl_val.l1_speed = auto_ctrl_val.l1_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.l1_cnt = 0.0;
	auto_ctrl_val.r0_speed = auto_ctrl_val.r0_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.r0_cnt = 0.0;
	auto_ctrl_val.r1_speed = auto_ctrl_val.r1_cnt / pulse_num_per_round * constant;
	auto_ctrl_val.r1_cnt = 0.0;
	
	if (auto_ctrl_val.l0_dir_state == FALSE)
	{
		auto_ctrl_val.l0_speed = -auto_ctrl_val.l0_speed;
	}
	
	if (auto_ctrl_val.l1_dir_state == FALSE)
	{
		auto_ctrl_val.l1_speed = -auto_ctrl_val.l1_speed;
	}

	if (auto_ctrl_val.r0_dir_state == FALSE)
	{
		auto_ctrl_val.r0_speed = -auto_ctrl_val.r0_speed;
	}
	
	if (auto_ctrl_val.r1_dir_state == FALSE)
	{
		auto_ctrl_val.r1_speed = -auto_ctrl_val.r1_speed;
	}
	
	float l_speed = (auto_ctrl_val.l0_speed + auto_ctrl_val.l1_speed) / 2.0;
	float r_speed = (auto_ctrl_val.r0_speed + auto_ctrl_val.r1_speed) / 2.0;
	
	auto_ctrl_val.velocity_feedback = (l_speed + r_speed) / 2.0;
	auto_ctrl_val.angular_feedback = (r_speed - l_speed) / two_wheel_dist;
	
	// calc target speed
	// get target speed
	static float last_angular;
	last_angular = auto_ctrl_val.angular_cmd;
  sscanf((char *)final_rx_frame, "%f %f", &auto_ctrl_val.velocity_cmd, &auto_ctrl_val.angular_cmd);
	if (last_angular == 0 && auto_ctrl_val.angular_cmd != 0)
	{
		clear_i();
	}
	
	check_limit(&auto_ctrl_val.velocity_cmd, &auto_ctrl_val.angular_cmd, &auto_ctrl_val.accel_cmd);
	
	float temp = (two_wheel_dist * auto_ctrl_val.angular_cmd) / 2.0;
	auto_ctrl_val.l0_target_speed = auto_ctrl_val.velocity_cmd - temp;
	auto_ctrl_val.l1_target_speed = auto_ctrl_val.velocity_cmd - temp;
	auto_ctrl_val.r0_target_speed = auto_ctrl_val.velocity_cmd + temp;
	auto_ctrl_val.r1_target_speed = auto_ctrl_val.velocity_cmd + temp;
	
	auto_ctrl_val.spd_pid_l0.cur = auto_ctrl_val.l0_speed;
	auto_ctrl_val.spd_pid_l1.cur = auto_ctrl_val.l1_speed;
	auto_ctrl_val.spd_pid_r0.cur = auto_ctrl_val.r0_speed;
	auto_ctrl_val.spd_pid_r1.cur = auto_ctrl_val.r1_speed;
	
//	arrange_transition_process(&auto_ctrl_val.l0_target_speed, auto_ctrl_val.spd_pid_l0.cur);
//	arrange_transition_process(&auto_ctrl_val.l1_target_speed, auto_ctrl_val.spd_pid_l1.cur);
//	arrange_transition_process(&auto_ctrl_val.r0_target_speed, auto_ctrl_val.spd_pid_r0.cur);
//	arrange_transition_process(&auto_ctrl_val.r1_target_speed, auto_ctrl_val.spd_pid_r1.cur);
	
	auto_ctrl_val.spd_pid_l0.target = auto_ctrl_val.l0_target_speed;
	auto_ctrl_val.spd_pid_l1.target = auto_ctrl_val.l1_target_speed;
	auto_ctrl_val.spd_pid_r0.target = auto_ctrl_val.r0_target_speed;
	auto_ctrl_val.spd_pid_r1.target = auto_ctrl_val.r1_target_speed;
	
	update_pid_param();
	pid_speed_control(auto_ctrl_val.velocity_cmd, auto_ctrl_val.angular_cmd);

}

/*********************************************
 *功能：电脑控制小车
 *********************************************/
void zhp_computer_control()
{                      
  auto_ctrl_val_update();
}
