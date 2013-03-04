#pragma config(Sensor, S1,     touchSensorL,         sensorTouch)
#pragma config(Sensor, S2,     touchSensorR,         sensorTouch)


#define FORWARD_SPEED 50
#define TURN_SPEED    10

void Anticlockwise(float a)
{
  nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;

  motor[motorC] = -TURN_SPEED;
  motor[motorB] = TURN_SPEED;

  float timeLeft = a*(float)TURN_CONST/(float)TURN_SPEED;
  for (; timeLeft > TIME_SPLIT; timeLeft -= TIME_SPLIT)
  {
    Wait();
  }
  wait1Msec(timeLeft);
  angle += a;

  motor[motorC] = 0;
  motor[motorB] = 0;
}


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

  motor[motorC] = 0;
  motor[motorB] = 0;
}


void Left30deg()
{
  Anticlockwise(PI/6.0);
}


void Right30deg()
{
  Clockwise(PI/6.0);
}


task main()
{
  while (true) {
   if (SensorValue(touchSensorL) != 0)   //a while loop is declared with the touchsensor's value being 0 as it true condition
   {
     motor[motorC] = -FORWARD_SPEED;                //motor C is run at a 100 power level
     motor[motorB] = -FORWARD_SPEED;
     wait1Msec(1000);
     Right30deg();
   }
   else if (SensorValue(touchSensorR) != 0)
   {
     motor[motorC] = -FORWARD_SPEED;                //motor C is run at a 100 power level
     motor[motorB] = -FORWARD_SPEED;
     wait1Msec(1000);
     Left30deg();
   }
   else
   {
      motor[motorC] = FORWARD_SPEED;                //motor C is run at a 100 power level
      motor[motorB] = FORWARD_SPEED;                //motor B is run at a 100 power level
   }


 }
}
