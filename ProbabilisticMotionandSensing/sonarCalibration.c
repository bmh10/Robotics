#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//



task main()
{
  float sensorDist;

  while (true) {

    sensorDist = SensorValue[sonarSensor];
    nxtDisplayStringAt(25, 25, "%f", sensorDist);
 }
}
