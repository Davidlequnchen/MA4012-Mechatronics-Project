#pragma config(Sensor, in5,    sharpSensor,    sensorAnalog)
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
		writeDebugStreamLine("Value sharp IR sensor1: %f", (SensorValue[sharpSensor]));
		writeDebugStreamLine("Value sharp IR sensor convert to cm: %f", (9200/(SensorValue[sharpSensor])));
		sleep(100);
	}

}
