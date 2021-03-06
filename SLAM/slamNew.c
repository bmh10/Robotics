#pragma config(Sensor, S1, touchSensorL, sensorTouch)
#pragma config(Sensor, S2, touchSensorR, sensorTouch)
#pragma config(Sensor, S4, sonarSensor, sensorSONAR)

#define NO_BINS 180
#define BOX_THRESHOLD 35

#define TURN_SPEED 20
#define FORWARD_SPEED 25
#define TURN_CONST 12012
#define SONAR_TURN_CONST 4.1
#define SONAR_TURN_EXTRA 0.94
#define FORWARD_CONST 2300//1980
#define PI 3.14159265358979
#define DESIRED_WALL_DIST 21
#define WALL_FOLLOW_K 0.3
#define SENSOR_MAX 100

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
    //int c = (angle < 0) ? -5 : 0;
    motor[motorA] = i*25;
    wait1Msec(i*angle*SONAR_TURN_CONST);
    motor[motorA] = 0;
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
 * Follows gap until gap found
 */
void ForwardWallFollow(bool sonarFacingLeft)
{
	writeDebugStreamLine("WALL_FOLLOW");

  int var;
  float sensorDist;
  float prevSensorDist = -1;
  int count = 0, count2 = 0, switchCount = 0;
  int lspeed, rspeed;
  bool switchL, switchR;
  //int timeLeft = (int) (distance*FORWARD_CONST/(float)FORWARD_SPEED);
  while (true)
  {
  	sensorDist = SensorValue[sonarSensor];
  	if (sensorDist > SENSOR_MAX) continue; // Ignore error readings

    if (switchL || switchR)
    	switchCount++;

    if (prevSensorDist == sensorDist) {
    	count2++;
    	if (count2 > 25) {
    	  if (!sonarFacingLeft && abs(lspeed) > FORWARD_SPEED) {
    		  switchR = true;
    		  switchL = false;
      	}
      	else if (sonarFacingLeft && abs(rspeed) > FORWARD_SPEED) {
      		switchR = false;
      		switchL = true;
      	}
        count2 = 0;
  		}
     }
     else count2 = 0;

  	// If difference between current and previous sensor reading is large, we have found a gap
  	if (abs(prevSensorDist - sensorDist) > 15 && prevSensorDist != -1)
  		count++; // Possibly found gap
    else {
    	count = 0;
    	prevSensorDist = sensorDist;
    }
    if (count > 7) // && abs(var) < 10)
    	break; // Almost definately found gap

    if (switchCount > 25)
    	switchR = false;
      switchL = false;
    }

    var = WALL_FOLLOW_K*(sensorDist - DESIRED_WALL_DIST);
    if (sonarFacingLeft || switchL) {
      lspeed = FORWARD_SPEED-var;
      rspeed = FORWARD_SPEED+var;
    }
    else if (!sonarFacingLeft || switchR) {
      lspeed = FORWARD_SPEED+var;
      rspeed = FORWARD_SPEED-var;
    }
    setMotorSpeeds(lspeed, rspeed);
    writeDebugStreamLine("MOTORS: %d %f", var, sensorDist);
    //wait1Msec(1);
  setMotorSpeeds(0, 0);
  PlayImmediateTone(500, 30);
}



/*---------------------------------------*/

void moveOutOfStartBlock()
{
	int i;
	int startIdx = -1;
  int endIdx = -1;
	int dists[NO_BINS];
	//Determine angle to turn
	float ang = (float) (360/NO_BINS)+SONAR_TURN_EXTRA;
	// Spin sonar and take readings
  for (i=0; i < NO_BINS; i++)
  {
    rotateSonar(ang);
    // Save dist in loc_sig instance ls
    dists[i] = SensorValue[sonarSensor];
    //writeDebugStreamLine("DIST: %d %d", i, dists[i]);

    if (dists[i] > BOX_THRESHOLD && dists[i] < SENSOR_MAX && startIdx == -1)
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
  wait1Msec(500);
  rotateSonar(-360);

  //writeDebugStreamLine("S: %d E: %d", startIdx, endIdx);

  int idx = (startIdx+endIdx)/2;
  float angleToTurn = idx*360/NO_BINS;
  writeDebugStreamLine("ANGLE: %f", angleToTurn);
  // Ensure we turn smallest distance possible
  angleToTurn -= 20;

  if (angleToTurn > 180)
  	angleToTurn = angleToTurn-360;

  if (angleToTurn < 0)
  	angleToTurn -= 10;

  Turn(angleToTurn);
  Forward(40);
}


int determineStartPosition()
{
	int leftDist, rightDist;
	rotateSonar(90);
	leftDist = SensorValue[sonarSensor];
	wait1Msec(500);
	rotateSonar(-180);
	rightDist = SensorValue[sonarSensor];
	wait1Msec(500);
	rotateSonar(90);

	if (leftDist > 100 && rightDist < 100) return 1;
	if (leftDist > 100 && rightDist > 100) return 2;
	if (leftDist < 100 && rightDist > 100) return 3;

  return 1;
}

void executePlan1()
{
  Turn(90);
  rotateSonar(90);
  Forward(40);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  Forward(40);
  AtWaypoint(); // Waypoint 2
  Forward(-40);
  Turn(-90);
  Forward(40);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  Forward(40);
  AtWaypoint(); // Waypoint 3
  Forward(-40);
  Turn(90);
  Forward(40);
  rotateSonar(-180);
  //Skip past middle gap
  ForwardWallFollow(false);
  Forward(50);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  Forward(40);
  AtWaypoint(); // Waypoint 1
}

void executePlan2()
{
	Turn(-90);
  rotateSonar(-90);
  Forward(40);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  Forward(40);
  AtWaypoint(); // Waypoint 1
  Forward(-40);
  Turn(-90);
  Forward(40);
  rotateSonar(180);
  // Skip past middle gap
  ForwardWallFollow(true);
  Forward(50);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  Forward(40);
  AtWaypoint(); // Waypoint 3
  Forward(-40);
  Turn(90);
  Forward(40);
  rotateSonar(-180);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  Forward(40);
  AtWaypoint(); // Waypoint 2
}

void executePlan3()
{
	Turn(-90);
  rotateSonar(-90);
  Forward(40);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  Forward(40);
  AtWaypoint(); // Waypoint 2
  Forward(-40);
  Turn(90);
  Forward(40);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  Forward(40);
  AtWaypoint(); // Waypoint 1
  Forward(-40);
  Turn(-90);
  Forward(40);
  rotateSonar(180);
  //Skip past middle gap
  ForwardWallFollow(true);
  Forward(50);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  Forward(40);
  AtWaypoint(); // Waypoint 3
}

task main()
{
	//ForwardWallFollow(true);
	//return;
writeDebugStreamLine("---------------------------------------");
  moveOutOfStartBlock();
  int startPos = determineStartPosition();
  writeDebugStreamLine("START POS: %d", startPos);

  switch (startPos)
  {
    case 1: executePlan1(); break;
    case 2: executePlan2(); break;
    case 3: executePlan3(); break;
  }

  wait10Msec(1000);
}
