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

void AtWaypoint()
{
	PlayImmediateTone(500, 30);
  wait1Msec(1000);
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
	int i = (distance < 0) ? -1 : 1;
	setMotorSpeeds(i*FORWARD_SPEED, i*FORWARD_SPEED);
	writeDebugStreamLine("FORWARD: %f", distance);

  int timeLeft = (int) (i*distance*FORWARD_CONST/(float)FORWARD_SPEED);
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
    writeDebugStreamLine("DIST: %d %d", i, dists[i]);

    if (dists[i] > BOX_THRESHOLD && startIdx == -1)
  	{
  		startIdx = i;
    }
    else if (startIdx != -1 && endIdx == -1 && dists[i] < BOX_THRESHOLD)
    {
  		endIdx = i;
    }
  }
  if (endIdx == -1)
  	endIdx = NO_BINS;

  // Unspin sonar back to start position
  rotateSonar(-360);

  writeDebugStreamLine("S: %d E: %d", startIdx, endIdx);

  int idx = (startIdx+endIdx)/2;
  float angleToTurn = idx*360/NO_BINS;
  writeDebugStreamLine("ANGLE: %f", angleToTurn);
  // Ensure we turn smallest distance possible
  //if (angleToTurn > 180)
  	//angleToTurn = angleToTurn-260;

  Turn(angleToTurn);
  Forward(40);
}


int determineStartPosition()
{
	int leftDist, rightDist;
	rotateSonar(90);
	leftDist = SensorValue[sonarSensor];
	rotateSonar(-180);
	rightDist = SensorValue[sonarSensor];
	rotateSonar(90);

	if (leftDist > 100 && rightDist < 100) return 1;
	if (leftDist > 100 && rightDist > 100) return 2;
	if (leftDist < 100 && rightDist > 100) return 3;

  return -1;
}

void executePlan1()
{
  Turn(90);
  rotateSonar(90);
  //TODO: Wall following here
  Forward(252);
  Turn(90);
  Forward(20);
  AtWaypoint(); // Waypoint 2
  Forward(-20);
  Turn(-90);
  Forward(252);
  Turn(90);
  Forward(20);
  AtWaypoint(); // Waypoint 3
  Forward(-20);
  Turn(90);
  Forward(504);
  Turn(-90);
  Forward(20);
  AtWaypoint(); // Waypoint 1
}

void executePlan2()
{
	Turn(360);
}

void executePlan3()
{
	Turn(360);
}

task main()
{
  moveOutOfStartBlock();
  int startPos = determineStartPosition();
  writeDebugStreamLine("START POS: %d", startPos);
  if (startPos == -1) return;

  switch (startPos)
  {
    case 1: executePlan1(); break;
    case 2: executePlan2(); break;
    case 3: executePlan3(); break;
  }

  wait10Msec(1000);
}
