#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)

#define NO_BINS 72
#define BOX_THRESHOLD 35

#define TURN_SPEED 20
#define FORWARD_SPEED 25
#define TURN_CONST 12012
#define FORWARD_CONST 1980
#define PI 3.14159265358979

/*
 * Converts radians to degrees
 */
float radToDeg(float a)
{
	return a*(180/PI);
}

float degToRad(float a)
{
	return a*(PI/180);
}

void setMotorSpeeds(int lspeed, int rspeed)
{
	motor[motorB] = lspeed;
  motor[motorC] = rspeed;
}

void rotateSonar(float angle)
{
    int i = (angle < 0) ? -1 : 1;
    motor[motorA] = i*25;
    wait1Msec(i*angle*4);
    motor[motorA] = 0;
}

void Forward(float distance)
{
	setMotorSpeeds(FORWARD_SPEED, FORWARD_SPEED);
	writeDebugStreamLine("FORWARD: %f", distance);

  int timeLeft = (int) (distance*FORWARD_CONST/(float)FORWARD_SPEED);
  wait1Msec(timeLeft);
  setMotorSpeeds(0, 0);
}

/*
 * +ve angles are anticlockwise, angle in degrees
 */
void Turn(float a)
{
  int i = (a < 0) ? -1 : 1;
  setMotorSpeeds(-i*TURN_SPEED, i*TURN_SPEED);
  writeDebugStreamLine("TURN: %f", a);

  // Correction for motor bias
  wait1Msec(i*degToRad(a)*(float)TURN_CONST/(float)TURN_SPEED);
  setMotorSpeeds(0, 0);
}

/*---------------------------------------*/

void moveOutOfStartBlock()
{
	int i;
	int startIdx = -1;
  int endIdx = -1;
	int dists[NO_BINS];
	//Determine angle to turn
	float ang = 360/NO_BINS+2.90;
	// Spin sonar and take readings
  for (i=0; i < NO_BINS; i++)
  {
    rotateSonar(ang);
    // Save dist in loc_sig instance ls
    dists[i] = SensorValue[sonarSensor];
    writeDebugStreamLine("DIST: %d", dists[i]);

    if (dists[i] > BOX_THRESHOLD && startIdx == -1)
  	{
  		startIdx = dists[i];
    }
    else if (startIdx != -1 && endIdx == -1 && dists[i] < BOX_THRESHOLD)
    {
  		endIdx = dists[i];
    }
  }
  // Unspin sonar back to start position
  rotateSonar(-360);

  writeDebugStreamLine("S: %d E: %d", startIdx, endIdx);

  int idx = (startIdx+endIdx)/2;
  float angleToTurn = idx*360/NO_BINS;
  Turn(angleToTurn);
  Forward(20);
}

int determineStartPosition()
{
	int leftDist, rightDist;
	rotateSonar(90);
	leftDist = SensorValue[sonarSensor];
	rotateSonar(-180);
	rightDist = SensorValue[sonarSensor];
	rotateSonar(90);


	return 0;
}

task main()
{

  moveOutOfStartBlock();
  int startPos = determineStartPosition();
  /*switch (startPos)
  {
    case 0: executePlan0(); break;
    case 1: executePlan1(); break;
    case 2: executePlan2(); break;
  }*/


}
