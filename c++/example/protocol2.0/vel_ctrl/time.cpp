#include <cstddef>
#include "time.h"

CTime::CTime() {
	gettimeofday(&t_init,NULL);
	t_prev = 0.;
}

double CTime::get_time() {
	gettimeofday(&t_now,NULL);
	t = (t_now.tv_sec - t_init.tv_sec);
	t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
	return t;
}

double CTime::get_delta_t() {
	gettimeofday(&t_now,NULL);
	t = (t_now.tv_sec - t_init.tv_sec);
	t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
	
	del_t = t-t_prev;
	t_prev = t;
	return del_t;
}
