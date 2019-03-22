#include "traj.h"

DVS_trajectory::DVS_trajectory(){
}

float DVS_trajectory::sample(float t, float freq)
{
//	printf("t: %f\n\n",t);
	return sin(t*freq);
}

float DVS_trajectory::random_sample(float t)
{
//	printf("t: %f\n\n",t);
	return sin(t*randn(0., 1.))*randn(1., 1.);
}

float DVS_trajectory::randn (float mu, float sigma)
{
  float U1, U2, W, mult;
  static float X1, X2;
  static int call = 0;
 
  if (call == 1)
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

