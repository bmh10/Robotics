
// Return a random number sampled uniformly from the range 0 to max
float sampleUniform(float max)
{
		// Converts random int to correct float range
    float a = ((float)(rand()) / (float) 32768);
    if (a < 0.0) a *= -1.0;
		return a * max;
		//65546
}

// Return a random number sampled from a Gaussian distribution with
// mean = mean, standard deviation = sigma
float sampleGaussian(float mean, float sigma)
{
  float u     = sampleUniform(1.0);
  float theta = sampleUniform(2 * PI);

   //writeDebugStreamLine("u=%f", u);


  // Fix to avoid infinity problem
  if (u == 0) {
    u = 0.0001;
  }

  float r = sqrt(-2*log(u));

  float x = r * cos(theta);



  return mean - sigma * x;
}
