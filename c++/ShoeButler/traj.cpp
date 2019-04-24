#include "traj.h"

DVS_trajectory::DVS_trajectory(){
}


//freq: Hz.
float DVS_trajectory::sample(float t, float freq)
{
//	printf("t: %f\n\n",t);
  double PI = 3.14159265359;
	return sin(t*2.*PI*freq);
}

float DVS_trajectory::random_sample(float t)
{
	//printf("randn1: %f\n",randn(0.,1.));
	//printf("randn2: %f\n\n",randn(0.,2.));
	return (randn(0., 1.));
}

float DVS_trajectory::randn (float mu, float sigma)
{
  float U1, U2, W, mult;
  static float X1, X2;
  static int call = 0;
 
  if (call)
    {
      call = !call;
      return (mu + sigma * (double) X2);
    }
 
  do
    {
      U1 = -1 + ((float) rand () / RAND_MAX) * 2;
      U2 = -1 + ((float) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);
 
  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;
 
  call = !call;
 
  return (mu + sigma * (float) X1);
}

