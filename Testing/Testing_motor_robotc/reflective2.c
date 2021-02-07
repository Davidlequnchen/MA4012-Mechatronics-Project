#pragma config(Sensor, in5,    reflect2,       sensorAnalog)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma DebuggerWindows("DebugStream")

task main()
{

//Clears Debug Stream before use
	clearDebugStream();

	//While there are at least 100 bytes available in the buffer
	while(getAvailSpaceInDebugStream() >100)
	{
		//Write the value of the Sonar sensor to it
		writeDebugStreamLine("Value : %f", (SensorValue[reflect2]));
		sleep(100);
	}

}
