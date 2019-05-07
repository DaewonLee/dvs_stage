
#ifndef CTIME_H_
#define CTIME_H_

#include <sys/time.h>

class CTime
{
public:
	CTime();
	~CTime(){}

	struct timeval t_init, t_now;
	double t, del_t, t_prev;
	double get_time();
	double get_delta_t();
	
};

#endif
