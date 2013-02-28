#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)

/*
 * Robotics Practical 6: Monte Carlo Localisation
 *                 (Group 16)
 */

#include "sample.h"

#define X 0
#define Y 1
#define ANGLE 2
#define WEIGHT 3

#define TURN_SPEED 20
#define FORWARD_SPEED 25
#define TURN_CONST 12012
#define FORWARD_CONST 1980
#define PI 3.14159265358979

#define SCREEN_MAX_X 99
#define SCREEN_MAX_Y 63
#define DISPLAY_SCALE 0.3

#define NUM_PARTICLES 100
#define K 0.01
#define VARIANCE 2
#define MAX_ANGLE 10000
#define ACCURACY 4
#define SONAR_SYSTEMATIC_ERROR 1.5
#define SONAR_DIST_FROM_WHEELBASE 1
#define SONAR_ERROR 250
#define DIST_PER_MOTION 20
#define UPDATES_PER_MOTION 5

const int NUM_WALLS = 8;
const int NUM_WAYPOINTS = 8;

//                           a    b    c    d    e    f    g    h
float wallAx[NUM_WALLS] = {  0,   0,  84,  84, 168, 168, 210, 210};
float wallAy[NUM_WALLS] = {  0, 168, 126, 210, 210,  84,  84,   0};
float wallBx[NUM_WALLS] = {  0,  84,  84, 168, 168, 210, 210,   0};
float wallBy[NUM_WALLS] = {168, 168, 210, 210,  84,  84,   0,   0};

float waypoints[NUM_WALLS][2] = {{180,30},{180,54},{126,54},{126,168},{126,126},{30,54},{84,54},{84,30}};

float particles[NUM_PARTICLES][4]; // x, y, theta, weight
float newParticles[NUM_PARTICLES][4];
float cumWeights[NUM_PARTICLES];

// screen size = (100 x 64)

// Robot's pose
float robotx = 84, roboty = 30, robotangle = 0;
float sonarReading;
bool sonarError = false;

/*---------------------WRAPPER FUNCTIONS---------------------------*/
/*
 * Converts radians to degrees
 */
float radToDeg(float a)
{
	return a*(180/PI);
}

float _atan2(float dx, float dy)
{
   float phi;   //phi=radians;

   if (dx>0) {phi=atan(dy/dx);}
   else if ((dx<0)&&(dy>=0))  {phi=PI+atan(dy/dx);}
   else if ((dx<0)&&(dy<0))   {phi=-PI+atan(dy/dx);}
   else if ((dx==0)&&(dy>0))  {phi=PI/2;}
   else if ((dx==0)&&(dy<0))  {phi=-PI/2;}
   else if ((dx==0)&&(dy==0)) {phi=0;}

   return phi;
}

void drawParticle(int x, int y)
{
	nxtSetPixel(x*DISPLAY_SCALE, y*DISPLAY_SCALE);
}

void drawMap()
{
	int i;
	// Draw walls
  for (i = 0; i < NUM_WALLS; i++)
  {
    nxtDrawLine(wallAx[i]*DISPLAY_SCALE, wallAy[i]*DISPLAY_SCALE, wallBx[i]*DISPLAY_SCALE, wallBy[i]*DISPLAY_SCALE);
  }
  // Draw waypoints
  for (i = 0; i < NUM_WAYPOINTS; i++)
  {
    nxtDrawCircle((waypoints[i][X])*DISPLAY_SCALE, (waypoints[i][Y])*DISPLAY_SCALE, 4);
  }
}

void drawStats()
{
  // Print new average position to screen
  nxtDisplayStringAt(SCREEN_MAX_X-35, SCREEN_MAX_Y-10, "X:%d", robotx);
  nxtDisplayStringAt(SCREEN_MAX_X-35, SCREEN_MAX_Y-20, "Y:%d", roboty);
  nxtDisplayStringAt(SCREEN_MAX_X-35, SCREEN_MAX_Y-30, "A:%f", radToDeg(robotangle));
  nxtDisplayStringAt(SCREEN_MAX_X-35, SCREEN_MAX_Y-40, "S:%f", sonarReading);
}

void setMotorSpeeds(int lspeed, int rspeed)
{
	motor[motorB] = lspeed;
  motor[motorC] = rspeed;
}

float getSonarReading()
{
	float a = SensorValue[sonarSensor];
	float b = SensorValue[sonarSensor];
	float c = SensorValue[sonarSensor];
  return (a+b+c)/3.0 - SONAR_SYSTEMATIC_ERROR + SONAR_DIST_FROM_WHEELBASE;
}

void copyNewParticlesToParticles()
{
  for (int i=0; i < NUM_PARTICLES; i++)
  {
    for (int j=0; j < 4; j++)
    {
      particles[i][j] = newParticles[i][j];
    }
  }
}

void copyParticlesToNewParticles()
{
  for (int i=0; i < NUM_PARTICLES; i++)
  {
    for (int j=0; j < 4; j++)
    {
      newParticles[i][j] = particles[i][j];
    }
  }
}

/*---------------------END WRAPPER FUNCTIONS-----------------------*/

task DrawParticles()
{
	eraseDisplay();
	drawMap();
	drawStats();

	//drawParticle((int) robotx, (int) roboty);

  for (int i=0; i < NUM_PARTICLES; i++)
  {
    nxtSetPixel((int)(particles[i][X]*DISPLAY_SCALE),
                (int)(particles[i][Y]*DISPLAY_SCALE));
  }
}

void calculateMeanPosition()
{
  float mx = 0, my = 0, mt = 0;

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    mx += particles[i][X]*particles[i][WEIGHT];
    my += particles[i][Y]*particles[i][WEIGHT];
    mt += particles[i][ANGLE]*particles[i][WEIGHT];
  }

  robotx = mx;
  roboty = my;
  robotangle = mt;
}

/*---------------------1. MOTION PREDICTION BASED ON ODOMETRY-----------------------*/

void updateParticlesForward(float dist)
{
  float e, f;

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
  	// If distance is less than 20 or sonar not working thsn don't add any randomness to stop overshooting
    e = (dist < DIST_PER_MOTION || sonarError) ? 0 : sampleGaussian(0, 3.0);
    f = (dist < DIST_PER_MOTION || sonarError) ? 0 : sampleGaussian(0, 0.005);
    particles[i][X] += (dist + e) * cos(particles[i][ANGLE]);
    particles[i][Y] += (dist + e) * sin(particles[i][ANGLE]);
    particles[i][ANGLE] += f;
  }

  StartTask(DrawParticles);
}

void updateParticlesTurn(float a)
{
  float g;
  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    g = (sonarError) ? 0 : sampleGaussian(0.0, 0.025);
    particles[i][ANGLE] += (a + g);
    // Ensure angle is in range -2PI < theta < 2PI
    //while (particles[i][ANGLE] < -2*PI) particles[i][ANGLE] += 2*PI;
    //while (particles[i][ANGLE] > 2*PI) particles[i][ANGLE] -= 2*PI;
  }
}

/*---------------------2. MEASUREMENT UPDATE BASED ON SONAR-----------------------*/

/*
 * Likelihood Function (TASK 1 - ASSESSED)
 */
float calculate_likelihood(float x, float y, float theta, float z)
{
	float smallestm = 10000;
  float top, bot, m, xco, yco, p;
  int wallIdx = -1;
  float beta;

  // Find which wall sonar beam would hit given the robot's current pose
  for (int i=0; i < NUM_WALLS; i++)
  {
    top = (wallBy[i]-wallAy[i])*(wallAx[i] - x) - (wallBx[i]-wallAx[i])*(wallAy[i]-y);
    bot = (wallBy[i]-wallAy[i])*cos(theta) - (wallBx[i]-wallAx[i])*sin(theta);
    m = top/bot;
    xco = x + m*cos(theta);
    yco = y + m*sin(theta);


    if (m > 0 && m < smallestm
        && ((wallAx[i] > wallBx[i] && xco <= wallAx[i] && xco >= wallBx[i])
        || (wallAx[i] <= wallBx[i] && xco >= wallAx[i] && xco <= wallBx[i]))
        && ((wallAy[i] > wallBy[i] && yco <= wallAy[i] && yco >= wallBy[i])
        || (wallAy[i] <= wallBy[i] && yco >= wallAy[i] && yco <= wallBy[i])))
    {
      smallestm = m;
      wallIdx = i;
    }
  }

  if (wallIdx == -1)
    return -1.0;

  // Check incident angle is less than acceptable angle to get accurate sonar reading
  top = cos(theta)*(wallAy[wallIdx]-wallBy[wallIdx]) + sin(theta)*(wallBx[wallIdx]-wallAx[wallIdx]);
  bot = sqrt(pow(wallAy[wallIdx]-wallBy[wallIdx], 2) + pow(wallBx[wallIdx]-wallAx[wallIdx], 2));
  beta = acos(top/bot);

  //if (beta > MAX_ANGLE) {
    //return -1.0;
  //}

  p = (sonarReading-smallestm)*(smallestm-sonarReading);
  float bias = (25-radToDeg(beta)) / 25; // MAX_ANGLE = 25
  float ret = exp(p/(2*VARIANCE))*bias + K;

  return ret;
}

/*---------------------3. NORMALISATION-----------------------*/

void normalise(float sumWeights)
{
  for (int i = 0; i < NUM_PARTICLES; i++)
    particles[i][WEIGHT] /= sumWeights;
}

/*---------------------4. RESAMPLING-----------------------*/

void resample()
{
  cumWeights[0] = particles[0][WEIGHT];

  for (int i = 1; i < NUM_PARTICLES; i++)
    cumWeights[i] = cumWeights[i-1] + particles[i][WEIGHT];

  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    float r = sampleUniform(1.0);

    int index = NUM_PARTICLES-1;

    for (int j = 0; j < NUM_PARTICLES; j++)
    {
      if (r <= cumWeights[j]){
        index = j;
        break;
      }
    }

    newParticles[i][X] = particles[index][X];
    newParticles[i][Y] = particles[index][Y];
    newParticles[i][ANGLE] = particles[index][ANGLE];
    newParticles[i][WEIGHT] = 1/(float)NUM_PARTICLES;
  }

  // Replace them with the new ones
  copyNewParticlesToParticles();
}

/*
 * At the end of each motion, make sonar measurement,
 * adjust particle weights based on likelihood function,
 * then normalise the weights and resample.
 */
void postMotionUpdate()
{
	float likelihood, particlesWithError = 0, sumWeights = 0;

	// Sonar reading with corrections
  sonarReading = getSonarReading();
  // If cannot get sonar reading update position based on particles alone
  if (sonarReading > SONAR_ERROR)
  {
  	sonarError = true;
  	writeDebugStreamLine("SONAR READING ERROR");
  }
  sonarError = false;

  // Replace them with the new ones
  //copyParticlesToNewParticles();

  // Adjust particle weights based on likelihood function
  for (int i = 0; i < NUM_PARTICLES; i++)
  {
  	 likelihood = calculate_likelihood(particles[i][X], particles[i][Y], particles[i][ANGLE], sonarReading);

  	 if (likelihood != -1)
       particles[i][WEIGHT] *= likelihood;
     else
       particlesWithError++;
     sumWeights += particles[i][WEIGHT];
  }

  // If more than 20% of particles report an incident angle > sonar is capable of reading
  // then revert weights and skip update.
  //if (particlesWithError > NUM_PARTICLES/5) {
  //	calculateMeanPosition();
  //	copyNewParticlesToParticles();
  //	writeDebugStreamLine("LARGE INCIDENT ANGLE - SKIP UPDATE");
  //	return;
  //}

  normalise(sumWeights);
  resample();
  calculateMeanPosition();
  StartTask(DrawParticles);
}

void Forward(float distance)
{
	setMotorSpeeds(FORWARD_SPEED, FORWARD_SPEED);
	writeDebugStreamLine("FORWARD: %f", distance);

  int timeLeft = (int) (distance*FORWARD_CONST/(float)FORWARD_SPEED);
  int updatePeriod = timeLeft/UPDATES_PER_MOTION;
  float distPerUpdate = distance/UPDATES_PER_MOTION;
  for (; timeLeft > 0; timeLeft--)
  {
  	if (timeLeft % updatePeriod == 0)
  	{
  		updateParticlesForward(distPerUpdate);
  		distance -= distPerUpdate;
  	}
  	wait1Msec(1);
  }
  setMotorSpeeds(0, 0);

  // Update any remaining distance to particles
  if (distance > 0)
    updateParticlesForward(distance);
}

/*
 * +ve angles are anticlockwise
 */
void Turn(float a)
{
  int i = (a < 0) ? -1 : 1;
  setMotorSpeeds(-i*TURN_SPEED, i*TURN_SPEED);
  writeDebugStreamLine("TURN: %f", radToDeg(a));

  // Correction for motor bias
  float deg = radToDeg(a);
  if (i==-1 && deg > -95.0 && deg < 95.0)
    wait1Msec(i*a*1.20*(float)TURN_CONST/(float)TURN_SPEED);
  else
    wait1Msec(i*a*(float)TURN_CONST/(float)TURN_SPEED);
  updateParticlesTurn(a);
  setMotorSpeeds(0, 0);
}

/*
 * Makes movement towards (wx, wy), though may not reach it
 * in one motion.
 */
void navigateToWaypoint(float wx, float wy)
{
  float dx, dy, a, dist, ang;

  dx = wx - robotx;
  dy = wy - roboty;
  a = _atan2(dx, dy);
  ang = a - robotangle;

  while (ang < -PI) ang += 2*PI;
  while (ang > PI) ang -= 2*PI;

  Turn(ang);

  dist = sqrt(dx*dx + dy*dy);

  if (dist > DIST_PER_MOTION)
    Forward(DIST_PER_MOTION);
  else
    Forward(dist);

  // After each motion recalculate and normalise weights, then resample
  postMotionUpdate();

  writeDebugStreamLine("X: %f Y: %f A: %f S: %f", robotx, roboty, radToDeg(robotangle), sonarReading);
}

void init()
{
	// Initialise particles
	int i;
  for (i = 0; i < NUM_PARTICLES; i++)
  {
    particles[i][X] = robotx;
    particles[i][Y] = roboty;
    particles[i][ANGLE] = robotangle;
    particles[i][WEIGHT] = 1.0 / (float) NUM_PARTICLES;
  }
  drawMap();
  postMotionUpdate();

  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
}

task main()
{
	writeDebugStreamLine("---------------------------------------------------------");
  init();

  for (int i = 0; i < NUM_WALLS; i++)
  {
    while (robotx < waypoints[i][0]-ACCURACY || robotx > waypoints[i][0]+ACCURACY
        || roboty < waypoints[i][1]-ACCURACY || roboty > waypoints[i][1]+ACCURACY)
    {
      navigateToWaypoint(waypoints[i][0], waypoints[i][1]);
    }
    // Waypoint reached
    PlayImmediateTone(500, 30);
    nxtDisplayStringAt(SCREEN_MAX_X-30, SCREEN_MAX_Y-50, "= %d =", i+1);
    writeDebugStreamLine("---------------------WAYPOINT %d REACHED---------------------\n", i+1);
  }

  wait1Msec(10000);
}
