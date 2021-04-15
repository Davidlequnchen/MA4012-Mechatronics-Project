#pragma config(Sensor, in3,    LeftDistanceSensor, sensorAnalog)
#pragma config(Sensor, in4,    enemyDistanceSensor, sensorAnalog)
#pragma config(Sensor, in6,    gripperBumper,  sensorAnalog)
#pragma config(Sensor, dgtl2,  digital2,       sensorTouch)
#pragma config(Sensor, dgtl3,  digital3,       sensorTouch)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Opens Debug Stream Window automatically
#pragma DebuggerWindows("DebugStream")


task main()
{

  //Clears Debug Stream before use
	clearDebugStream();

	//While there are at least 100 bytes available in the buffer
	while(getAvailSpaceInDebugStream() >100)
	{
		//Write the value of the Sonar sensor to it
		writeDebugStreamLine("Analog pin test: %f", (SensorValue[gripperBumper]));
	  //writeDebugStreamLine("Left Bumper test: %f", (SensorValue(BackLeftSwitch)));
	  //writeDebugStreamLine("digital2 test: %f", (SensorValue(digital2)));
	  //writeDebugStreamLine("digital3 test: %f", (SensorValue(digital3)));
		sleep(100);
	}

}
