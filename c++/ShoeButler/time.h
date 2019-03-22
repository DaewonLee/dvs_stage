#ifndef CTIME_H_
#define CTIME_H_

#include <sys/time.h>

class CTime
{
public:
	CTime();
	~CTime(){}

	struct timeval t_init, t_now, t_alarm_init, t_alarm_now;
	double t, del_t, t_prev, t_alarm;
	double get_time();
	double get_delta_t();
	void initialize();
	void alarmReset();
	int alarm(float target);
	
};

#endif
