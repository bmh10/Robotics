#pragma config(Sensor, S1, touchSensorL, sensorTouch)
#pragma config(Sensor, S2, touchSensorR, sensorTouch)
#pragma config(Sensor, S4, sonarSensor, sensorSONAR)

#define NO_BINS 180
#define BOX_THRESHOLD 35

#define TURN_SPEED 20
#define FORWARD_SPEED 25
#define TURN_CONST 12490//12012
#define SONAR_TURN_CONST 4.1
#define SONAR_TURN_EXTRA 0.92
#define FORWARD_CONST 2300//1980
#define PI 3.14159265358979
#define DESIRED_WALL_DIST 26
#define DESIRED_FINISH_DIST 13
#define WALL_FOLLOW_K 0.6
#define WALL_FOLLOW_DIST 190
#define SENSOR_MAX 100

/*
 * Converts radians to degrees
 */
float radToDeg(float a)
{
	return a*(180/PI);
}

void playTone(int n)
{
	for (int i=0; i < n; i++) {
    PlayImmediateTone(500, 30);
    wait1Msec(500);
  }
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
    int c = (angle < 0) ? -5 : 0;
    motor[motorA] = i*25;
    wait1Msec(i*(angle+c)*SONAR_TURN_CONST);
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

void ForwardDoubleSpeed(float distance)
{
	int i = (distance < 0) ? -1 : 1;
	setMotorSpeeds(i*2*FORWARD_SPEED, i*2*FORWARD_SPEED);
	writeDebugStreamLine("FORWARD: %f", distance);

  int timeLeft = (int) (i*distance*FORWARD_CONST/(float)FORWARD_SPEED);
  wait1Msec(timeLeft);
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

void MoveToWaypoint()
{
	float sensorDist = SensorValue[sonarSensor];

	while (sensorDist > DESIRED_FINISH_DIST) {
	  setMotorSpeeds(FORWARD_SPEED, FORWARD_SPEED);
	  sensorDist = SensorValue[sonarSensor];
  }
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
  int count = 0;//, errorCount = 0;
  int lspeed, rspeed;
  //int timeLeft = (int) (distance*FORWARD_CONST/(float)FORWARD_SPEED);
  while (true)
  {
  	sensorDist = SensorValue[sonarSensor];
  	if (sensorDist > SENSOR_MAX || sensorDist < 5 ) {
  		//errorCount++;
  		// If robot cannot get a reading for a period of time roate sonar gradually
  		//if (errorCount > 50) {
  			 //setMotorSpeeds(2*lspeed, 2*rspeed);
  		  //if (sonarFacingLeft) rotateSonar(1);
  		  //else rotateSonar(-1);
  		  //prevSensorDist = sensorDist;
  	  //}
  	  continue; // Ignore error readings
    }
    //errorCount = 0;

  	// If difference between current and previous sensor reading is large, we have found a gap
  	if (abs(prevSensorDist - sensorDist) > 25 && prevSensorDist != -1 && sensorDist > 54) {
  		count++; // Possibly found gap
  		writeDebugStreamLine("BREAK: %f %f", sensorDist, prevSensorDist);
  	}
    else {
    	count = 0;
    	prevSensorDist = sensorDist;
    }
    if (count > 20) // && abs(var) < 10)
    	break; // Almost definately found gap

    //float sonarDiff = sensorDist - DESIRED_WALL_DIST;
    var = (sonarFacingLeft) ? WALL_FOLLOW_K*(sensorDist - DESIRED_WALL_DIST)
                            : WALL_FOLLOW_K*(sensorDist - (DESIRED_WALL_DIST));
    if (var > 1) var = 1;
    else if (var < -1) var = -1;
    if (sonarFacingLeft) {
    	//if (abs(sonarDiff) > 5) {
    		//sonarAngle -= var*0.1;
    	  //rotateSonar(-var*0.05);
    	//}
      lspeed = FORWARD_SPEED-var;
      rspeed = FORWARD_SPEED+var;
    }
    else if (!sonarFacingLeft) {
    	//if (abs(sonarDiff) > 5) {
    		//sonarAngle += var*0.1;
    	  //rotateSonar(var*0.05);
    	//}
      lspeed = FORWARD_SPEED+var;
      rspeed = FORWARD_SPEED-var;
    }
    setMotorSpeeds(2*lspeed, 2*rspeed);
    //writeDebugStreamLine("MOTORS: %d %f", var, sensorDist);
    //wait1Msec(1);
  }
  setMotorSpeeds(0, 0);
  //rotateSonar(-sonarAngle);
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
  wait1Msec(500);
  rotateSonar(-360);

  //writeDebugStreamLine("S: %d E: %d", startIdx, endIdx);

  int idx = (startIdx+endIdx)/2;
  float angleToTurn = idx*360/NO_BINS;
  writeDebugStreamLine("ANGLE: %f", angleToTurn);
  // Ensure we turn smallest distance possible
  //angleToTurn -= 20;

  if (angleToTurn > 180)
  	angleToTurn = angleToTurn-360;

  //if (angleToTurn < 0)
  	//angleToTurn -= 10;

  Turn(angleToTurn);
  Forward(39);
}


int determineStartPosition()
{
	int leftDist, rightDist;
	rotateSonar(100);
	leftDist = SensorValue[sonarSensor];
	wait1Msec(500);
	rotateSonar(-200);
	rightDist = SensorValue[sonarSensor];
	wait1Msec(500);
	rotateSonar(100);

	if (leftDist > 100 && rightDist < 100) return 1;
	if (leftDist > 100 && rightDist > 100) return 2;
	if (leftDist < 100 && rightDist > 100) return 3;

  return 1;
}

void executePlan1()
{
	//Correct angle
  Turn(80);
  rotateSonar(60);
  Forward(25);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  rotateSonar(-60);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 2
  Forward(-40);
  Turn(-90);
  Forward(40);
  rotateSonar(60);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  rotateSonar(-60);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 3
  Forward(-40);
  Turn(90);
  Forward(40);
  rotateSonar(-60);
  //Skip past middle gap
  ForwardWallFollow(false);
  ForwardDoubleSpeed(30);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  rotateSonar(60);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 1
}

void executePlan2()
{
	// Correct angle
	Turn(-100);
  rotateSonar(-70);
  Forward(25);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  rotateSonar(70);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 1
  Forward(-40);
  Turn(-90);
  Forward(40);
  rotateSonar(70);
  // Skip past middle gap
  ForwardWallFollow(true);
  ForwardDoubleSpeed(30);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  rotateSonar(-70);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 3
  Forward(-40);
  Turn(90);
  Forward(40);
  rotateSonar(-70);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  rotateSonar(70);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 2
}

void executePlan3()
{
	// Correct angle
	Turn(-100);
  rotateSonar(-70);
  Forward(25);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  rotateSonar(70);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 2
  Forward(-40);
  Turn(90);
  Forward(40);
  rotateSonar(-70);
  ForwardWallFollow(false);
  Forward(25);
  Turn(-90);
  rotateSonar(70);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 1
  Forward(-40);
  Turn(-90);
  Forward(40);
  rotateSonar(70);
  //Skip past middle gap
  ForwardWallFollow(true);
  ForwardDoubleSpeed(30);
  ForwardWallFollow(true);
  Forward(25);
  Turn(90);
  rotateSonar(-70);
  MoveToWaypoint();
  AtWaypoint(); // Waypoint 3
}

task main()
{
	//MoveTowardsWall(DESIRED_FINISH_DIST);
	//ForwardWallFollow(true);
		//wait10Msec(100);
  //Turn(90);
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
