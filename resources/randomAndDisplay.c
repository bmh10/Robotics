//*!!Sensor,    S1,          sonarSensor, sensorSONAR9V,      ,                  !!*//
// Example for using random number generator; also displaying pixels on NXT
#include "sample.h"

task main()
{
	eraseDisplay();

	// Array for accumulating Gaussian values
	int accumulator[100];
	for (int i = 0; i < 99; i++) {
		accumulator[i] = 0;
	}

	// Infinite loop
	while(1)
	{
		// Get and print uniform-distributed and Gaussian-distributed random numbers
		float uniform_float = sampleUniform(1.0);
		float gaussian_float = sampleGaussian(0.0, 1.0);
		nxtDisplayString(1, "Uniform : %04f", uniform_float);
		nxtDisplayString(2, "Gaussian: %04f", gaussian_float);

		// Build a histogram of Gaussian numbers and plot on the NXT screen

		// Scale random number to screen resolution
		int gaussian_int = (int) (gaussian_float * 10.0) + 50;
		accumulator[gaussian_int]++;
		nxtSetPixel(gaussian_int, accumulator[gaussian_int]);

		wait1Msec(10);
	}

}
