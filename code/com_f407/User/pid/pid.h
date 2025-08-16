#ifndef __PID_H__
#define __PID_H__

#define	PID_ALGRITHM	1		// 1表示位置式	2表示增量式

/*pid*/
typedef struct
{
  float target_val;     //目标值
	float actual_val;     //实际值
	float Kp, Ki, Kd;     //定义比例、积分、微分系数
	
	float err;            //定义当前偏差值
	float err_last;       //定义最后一个偏差值
	
#if (PID_ALGRITHM==1)	// 位置式
	float integral;				// 累积
#elif (PID_ALGRITHM==2)	// 增量式
	float err_next;       //定义下一个偏差值
#endif
}pid_t;


extern void		PID_param_init(pid_t *pid, float Kp, float Ki, float Kd);
extern float	PID_Core(pid_t *pid);
extern void		PID_set_pid(pid_t *pid, float p, float i, float d);
extern void		PID_set_target(pid_t *pid, int target);
extern void		PID_set_actual(pid_t *pid, float actual_value);
extern void		PID_get_pid(pid_t *pid, float *buf);
extern void		PID_control(void);

#endif

