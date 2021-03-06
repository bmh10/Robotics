//#pragma config(Sensor, S1,  , sensorLightActive)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define TURN_SPEED 20
#define FORWARD_SPEED 25
#define TURN_CONST 12500
#define FORWARD_CONST 1800
#define PI 3.141592
#define DISPLAY_SCALE_SQUARE 0.900
#define DISPLAY_SCALE_PATH_FOLLOW 1.20
#define TIME_SPLIT 1
#define NUM_PARTICLES 100
#define PATH_FOLLOWING true
#define DRAW_DIST 10

#include "sample.h"

/*
struct Particle {
  float x;
  float y;
  float theta;
  float weight;
};
*/

float particles[NUM_PARTICLES][4]; //x, y, theta, weight

// screen size = (100 x 64)

float x = 0, y = 0, angle = 0;
float DISPLAY_SCALE  = DISPLAY_SCALE_SQUARE;
float yoff = 5*DISPLAY_SCALE, xoff = 25*DISPLAY_SCALE;


void updateParticlesForward(float dist)
{
  float e, f, theta;
  float D = dist;

  for (int i=0; i < NUM_PARTICLES; i++)
  {
    e = sampleGaussian(0.0, 0.01);
    f = sampleGaussian(0.0, 0.02);
    theta = particles[i][2];
    particles[i][0] += (D+e)*cos(theta);
    particles[i][1] += (D+e)*sin(theta);
    particles[i][2] += f;
  }
}

void updateParticlesRotate(float a)
{
  float g;
  for (int i=0; i < NUM_PARTICLES; i++)
  {
    g = sampleGaussian(0.0, 0.01);
    particles[i][2] += (a + g);
  }
}

/*
 * Draws pixels to screen. Note screen coords are flipped so that start orientation appears same as robot's
 * start orientation.
 */
void Draw()
{
  nxtSetPixel((int)(99 - ((y+xoff)/DISPLAY_SCALE)), (int)(63 - ((x+yoff)/DISPLAY_SCALE)));
}

void DrawParticles()
{
  for (int i=0; i < NUM_PARTICLES; i++)
  {
    nxtSetPixel((int)(99 - ((particles[i][1]+xoff)/DISPLAY_SCALE)), (int)(63 - ((particles[i][0]+yoff)/DISPLAY_SCALE)));
  }
}

void Wait()
{
  wait1Msec(TIME_SPLIT);
}


void Forward(float distance)
{
  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;


  motor[motorC] = FORWARD_SPEED;
  motor[motorB] = FORWARD_SPEED;
  int timeLeft = (int) (distance*FORWARD_CONST/(float)FORWARD_SPEED);
  int timeToTravelEdge = timeLeft;
  float remainingDist = distance;
  float distPerTimeSplit = distance/timeToTravelEdge;
  int timeFor10cms = (int) (10*FORWARD_CONST/(float)FORWARD_SPEED);
  for (; timeLeft > TIME_SPLIT; timeLeft -= TIME_SPLIT)
  {
    Wait();

    x+=cos(angle)*distPerTimeSplit;
    y+=sin(angle)*distPerTimeSplit;

    if (timeLeft % timeFor10cms == 0)
    {
      updateParticlesForward(10);
      DrawParticles();
      remainingDist -= 10.0 ;
    }

    Draw();
  }

   if (remainingDist > 0)
   {
    updateParticlesForward(remainingDist);
    DrawParticles();
   }

  motor[motorC] = 0;
  motor[motorB] = 0;
}

void Forward40cm()
{
  Forward(40);
}

/*
void Backward(float distance)
{
  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;

  motor[motorC] = -FORWARD_SPEED;
  motor[motorB] = -FORWARD_SPEED;

  float timeLeft = distance*FORWARD_CONST/(float)FORWARD_SPEED;
  for (; timeLeft > TIME_SPLIT; timeLeft -= TIME_SPLIT)
  {
    Wait();
    x-=cos(angle)/1000;
    y-=sin(angle)/1000;

    Draw();
  }

  motor[motorC] = 0;
  motor[motorB] = 0;
}

void Backward40cm()
{
  Backward(40);
}
*/

/*
 * +ve angles are clockwise
 */
void Turn(float a)
{
   //nxtDisplayStringAt(20, 50, "%f", a);
  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
  int i = (a < 0) ? -1 : 1;
  motor[motorC] = i*TURN_SPEED;
  motor[motorB] = -i*TURN_SPEED;

  float timeLeft = i*a*(float)TURN_CONST/(float)TURN_SPEED;
  for (; timeLeft > TIME_SPLIT; timeLeft -= TIME_SPLIT)
  {
    Wait();
  }
  wait1Msec(timeLeft);
  angle += a;

  updateParticlesRotate(a);

  //nxtDisplayStringAt(20, 40, "%f", angle);

  motor[motorC] = 0;
  motor[motorB] = 0;
}

/*
void Clockwise(float a)
{
  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;

  motor[motorC] = TURN_SPEED;
  motor[motorB] = -TURN_SPEED;

  float timeLeft = a*(float)TURN_CONST/(float)TURN_SPEED;
  for (; timeLeft > TIME_SPLIT; timeLeft -= TIME_SPLIT)
  {
    Wait();
  }
  wait1Msec(timeLeft);
  angle -= a;
  updateParticlesRotate(-a);

  motor[motorC] = 0;
  motor[motorB] = 0;
}
*/

void Left90deg()
{
  Turn(-PI/2.0);
}

void Right90deg()
{
  Turn(PI/2.0);
}

void drawSquare()
{
  int i = 0;
  while( i < 4 )
  {
    Forward40cm();
    Right90deg();
    i++;
  }
}

void initParticles()
{
  for (int i=0; i < NUM_PARTICLES; i++)
  {
    particles[i][0] = x;
    particles[i][1] = y;
    particles[i][2] = angle;
    particles[i][3] = 1.0/100.0; //weight set to 1/NUM_PARTICLES for this week
  }
}

void navigateToWaypoint(float wx, float wy)
{
  float mx = 0, my = 0, mt = 0, dx = 0, dy = 0, a = 0, ang = 0, dist = 0;

  for (int i=0; i < NUM_PARTICLES; i++)
  {
    mx += particles[i][3]*particles[i][0];
    my += particles[i][3]*particles[i][1];
    mt += particles[i][3]*particles[i][2];
  }

  dx = wx-mx;
  dy = wy-my;
  a = (dx != 0) ? atan(dy/dx) : PI;
  ang = a + mt;

  //nxtDisplayStringAt(20, 40, "%f", dx);
  //nxtDisplayStringAt(20, 30, "%f", dy);
  //nxtDisplayStringAt(20, 20, "%f", ang);

  //Ensure -PI < ang <= PI
  while (ang < -PI)
  {
    ang += 2*PI;
  }
  while (ang > PI)
  {
    ang -= 2*PI;
  }

  if (ang < 0)
    ang += PI;


  Turn(ang);

  dist = sqrt(dx*dx + dy*dy);
  Forward(dist);

  wait1Msec(1000);

}

void runPathTest()
{
  navigateToWaypoint(50, 50);
  navigateToWaypoint(50, -20);
  navigateToWaypoint(0 , 0  );
}

task main()
{
  initParticles();
  if (PATH_FOLLOWING)
  {
    DISPLAY_SCALE  = DISPLAY_SCALE_PATH_FOLLOW;
    yoff = 10*DISPLAY_SCALE; xoff = 5*DISPLAY_SCALE;
    runPathTest();
  }
  else
  {
    DISPLAY_SCALE  = DISPLAY_SCALE_SQUARE;
    yoff = 5*DISPLAY_SCALE; xoff = 25*DISPLAY_SCALE;
    drawSquare();
  }

  wait1Msec(10000);
}
