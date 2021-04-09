#pragma config(Sensor, dgtl1,  startSwitch,    sensorTouch)
#pragma config(Sensor, dgtl4,  reflectiveFL,   sensorDigitalIn)
#pragma config(Sensor, dgtl5,  reflectiveFR,   sensorDigitalIn)
#pragma config(Sensor, dgtl6,  reflectiveBL,   sensorDigitalIn)
#pragma config(Sensor, dgtl7,  reflectiveBR,   sensorDigitalIn)
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
		//read the sensor value and print for debugging
		writeDebugStreamLine("Value of reflective sensor front left: %d", SensorValue[reflectiveFL]);
		if (SensorValue[reflectiveFL] == 0)
		{
		   writeDebugStreamLine("reflective sensor front left detect something! ");
	  }
	  else
	  {
	  	 writeDebugStreamLine("reflective sensor front left detect nothing! ");
	  }

		sleep(100);
	}

}
