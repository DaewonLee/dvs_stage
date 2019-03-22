#include <cstddef>
#include "saictime.h"

CTime::CTime() {
	gettimeofday(&t_init,NULL);
	gettimeofday(&t_alarm_init,NULL);
	t_prev = 0.;
}

double CTime::get_time() {
	gettimeofday(&t_now,NULL);
	t = (t_now.tv_sec - t_init.tv_sec);
	t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
	return t;
}

void CTime::initialize() {
	gettimeofday(&t_init,NULL);
	t_prev = 0.;
}

void CTime::alarmReset() {
	gettimeofday(&t_alarm_init,NULL);
}

int CTime::alarm(float target) {
	gettimeofday(&t_alarm_now,NULL);
	t_alarm = (t_alarm_now.tv_sec - t_alarm_init.tv_sec);
	t_alarm += (t_alarm_now.tv_usec - t_alarm_init.tv_usec)/1000000.;
	if (t_alarm > target) return 1;
	else return 0;
}

double CTime::get_delta_t() {
	gettimeofday(&t_now,NULL);
	t = (t_now.tv_sec - t_init.tv_sec);
	t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
	
	del_t = t-t_prev;
	t_prev = t;
	return del_t;
}
