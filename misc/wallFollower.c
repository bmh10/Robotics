#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*--------------------------------------------------------------------------------------------------------*\
|*                                                                                                        *|
|*                                   - Obstacle Detection with Sonar -                                    *|
|*                                            ROBOTC on NXT                                               *|
|*                                                                                                        *|
|*  This program runs your robot forward until it detects and obstacle, at which point it stops.          *|
|*                                                                                                        *|
|*                                        ROBOT CONFIGURATION                                             *|
|*    NOTES:                                                                                              *|
|*    1)  The Sonar Sensor should be somewhere on the FRONT of the robot, facing FORWARD.                 *|
|*                                                                                                        *|
|*    MOTORS & SENSORS:                                                                                   *|
|*    [I/O Port]              [Name]              [Type]              [Description]                       *|
|*    Port B                  motorB              NXT                 Right motor                         *|
|*    Port C                  motorC              NXT                 Left motor                          *|
|*    Port 4                  sonarSensor         Sonar Sensor        Front mounted, front facing         *|
\*---------------------------------------------------------------------------------------------------4246-*/

#define SPEED 25
#define K 0.15

task main()
{
  int desiredDist = 20;   // Create variable 'distance_in_cm' and initialize it to 20(cm).
  int sensorDist, var;


  while (true) {
    sensorDist = SensorValue[sonarSensor];
    var = K*(SensorValue[sonarSensor] - desiredDist);
     motor[motorB] = SPEED-var;
     motor[motorC] = SPEED+var;
 }
}
