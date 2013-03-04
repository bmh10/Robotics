#pragma config(Sensor, S1, touchSensorL, sensorTouch)
#pragma config(Sensor, S2, touchSensorR, sensorTouch)
#pragma config(Sensor, S3, lightSensorL, sensorLightActive)
#pragma config(Sensor, S4, lightSensorR, sensorLightActive)

#define FORWARD_SPEED     30
#define TURN_SPEED        20
#define THRESHOLD         35
#define K                 1
#define SENSOR_DIFFERENCE_1 5
#define SENSOR_DIFFERENCE_2 20
#define TURN_CONST        13100
#define FORWARD_CONST     2095
#define PI                3.141592


int lightL, lightR, touchL, touchR;
int lastHitDir = 1;
bool lightDetected, facingLight;
bool postScanDetection = true;

void turn(float a)
{
	nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	int dir = a < 0 ? -1 : 1;
	if (a < 0) a = -a;

	motor[motorC] = -dir*TURN_SPEED;
	motor[motorB] = dir*TURN_SPEED;
	wait1Msec(a*(float)TURN_CONST/(float)TURN_SPEED);
	motor[motorC] = 0;
	motor[motorB] = 0;
}

void rightDeg(int d)
{
	turn(-d*PI/180.0);
}

void leftDeg(int d)
{
	turn(d*PI/180.0);
}

void driveForward()
{
	motor[motorC] = FORWARD_SPEED;
	motor[motorB] = FORWARD_SPEED;
}

void driveBackward()
{
	motor[motorC] = -FORWARD_SPEED;
	motor[motorB] = -FORWARD_SPEED;
}

void turnTowardsLight()
{
	nxtDisplayStringAt(10, 30, "Follow...");
	motor[motorC] = K*(lightR-lightL);
	motor[motorB] = -K*(lightR-lightL);
	postScanDetection = false;
}

void scanForLight()
{
	nxtDisplayStringAt(10, 30, "Scan... ");
	rightDeg(2*lastHitDir);
	postScanDetection = true;
}

void avoidObstacle()
{
	driveBackward();
	wait1Msec(1000);

	if (touchL != 0)
	{
		lastHitDir = -1;
		rightDeg(45);
	}
	else if (touchR != 0)
	{
		lastHitDir = 1;
		leftDeg(45);
	}

	driveForward();
	wait1Msec(2000);
}

void readSensors()
{
	lightL = SensorValue[lightSensorL];
	lightR = SensorValue[lightSensorR];
	touchL = SensorValue[touchSensorL];
	touchR = SensorValue[touchSensorR];

  lightDetected = lightL > THRESHOLD || lightR > THRESHOLD;
  facingLight = (postScanDetection) ?
	              abs(lightR-lightL) < SENSOR_DIFFERENCE_1
	            : abs(lightR-lightL) < SENSOR_DIFFERENCE_2;
}

task main()
{
	while(true)
	{
		readSensors();

		if (touchL != 0 || touchR != 0)
		{
			avoidObstacle();
		}
		else if (lightDetected && facingLight)
		{
		  driveForward();
	  }
		else if (lightDetected && !facingLight)
		{
			turnTowardsLight();
		}
		else
		{
			//Default case
		  scanForLight();
	}
	}
}
