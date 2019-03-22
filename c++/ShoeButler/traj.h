#include <stdlib.h>
#include <stdio.h>
#include <cmath>


class DVS_trajectory
{
public:
	DVS_trajectory();
	~DVS_trajectory(){};
	
	
	float sample(float t, float freq);
	float random_sample(float t);
	float randn(float mu, float sigma);
	


};
