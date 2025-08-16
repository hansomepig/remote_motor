#include "pid/pid.h"
#include "tim.h"
#include <stdio.h>

extern pid_t pid;

/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init(pid_t *pid, float Kp, float Ki, float Kd)
{
	/* 初始化参数 */
#ifdef PID_DEBUG
	printf("PID_init begin \n");
#endif
	pid->target_val=0.0;				
	pid->actual_val=0.0;
	pid->err = 0.0;
	pid->err_last = 0.0;
	
#if (PID_ALGRITHM==1)	// 位置式
	pid->integral= 0;
#elif (PID_ALGRITHM==2)	// 增量式
	pid->err_next = 0.0;
#endif

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

#ifdef PID_DEBUG
	printf("PID_init end \n");
#endif
}

/**
  * @brief  PID算法实现
  * @param  val		目标值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float PID_Core(pid_t *pid) 
{
	/*计算目标值与实际值的误差*/
  pid->err = pid->target_val - pid->actual_val;
	
	/*PID算法实现*/
#if (PID_ALGRITHM==1)	// 位置式
	/*误差累积*/
	pid->integral += pid->err;
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral
										+ pid->Kd * (pid->err - pid->err_last);
	
	/*传递误差*/
	pid->err_last=pid->err;
#elif (PID_ALGRITHM==2)	// 增量式
	float increment_val = pid->Kp*(pid->err - pid->err_next) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_next + pid->err_last);
	pid->actual_val += increment_val;
	
	/*传递误差*/
	pid->err_last = pid->err_next;
	pid->err_next = pid->err;
#endif
	
	return pid->actual_val;
}

/**
  * @brief  定时器周期调用函数
  * @param  无
	*	@note 	无
  * @retval 需要用户自行实现
  */
void PID_control(void)
{
	static int flag=0;
	static int num=0;
	static int run_i=0;
	
	if(!flag)
	{
		float val=PID_Core(&pid);
#ifdef PID_DEBUG
		printf("val,%f;act,%f\n",set_point,val);	
#endif
		run_i++;
		
		float increment = pid.target_val - val;
		if( (increment > 0 ? increment : -increment) <= 1 ) {
			num++;
		} else {	//必须满足连续次数
			num=0;
			
#if (PID_ALGRITHM==1)	// 位置式
			// 实际测得 50%->143.6	90%->265.7	100%->305.0
			// 所以要设置转速为w需要设置定时器比较值大致为33.8w
			int set_speed = 33.8*val;
#elif	(PID_ALGRITHM==2)	// 增量式
			// 实际测得 50%->143.6	90%->265.7	100%->305.0
			// 所以将转速增大m大约需要将pwm增大0.035m%
			int set_speed = 3.5*increment + __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
#endif
			
			if(set_speed >= htim2.Instance->ARR) {			// 超出上限
				set_speed = htim2.Instance->ARR;
			} else if (set_speed < 0) {			// 低于下限
				set_speed = 0;
			}
			
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, set_speed);
		}
		
//		if(num>20)//稳定次数
//		{
//#ifdef PID_DEBUG
//			printf("PID算法运行%d 次后稳定\r\n",run_i);
//#endif
//			flag=1;
//		}
	}
}


void PID_set_pid(pid_t *pid, float p, float i, float d)
{
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;
}

void PID_set_target(pid_t *pid, int target)
{
	pid->target_val = target;
}

void PID_set_actual(pid_t *pid, float actual_value)
{
	pid->actual_val = actual_value;
}

void PID_get_pid(pid_t *pid, float *buf)
{
	buf[0] = pid->Kp;
	buf[1] = pid->Ki;
	buf[2] = pid->Kd;
}

