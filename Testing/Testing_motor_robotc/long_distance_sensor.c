#pragma config(Sensor, in2,    RightDistanceSensor, sensorAnalog)
#pragma config(Sensor, in3,    LeftDistanceSensor, sensorAnalog)
#pragma config(Sensor, in4,    enemyDistanceSensor, sensorAnalog)
#pragma config(Sensor, in5,    backDistanceSensor, sensorAnalog)
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
		writeDebugStreamLine("Value sharp IR sensor1: %f", (SensorValue[enemyDistanceSensor]));
		//writeDebugStreamLine("Value sharp IR sensor convert to cm: %f", (18000/(SensorValue[RightDistanceSensor]-100)));
		sleep(100);
	}

}
