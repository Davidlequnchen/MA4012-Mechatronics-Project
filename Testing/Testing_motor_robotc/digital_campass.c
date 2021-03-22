#pragma config(Sensor, dgtl8, compassSupply,  sensorDigitalOut)
#pragma config(Sensor, dgtl12,  compassWest,    sensorDigitalIn)
#pragma config(Sensor, dgtl11,  compassSouth,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compassEast,    sensorDigitalIn)
#pragma config(Sensor, dgtl9, compassNorth,   sensorDigitalIn)

#define NORTH 0
#define NORTHEAST 1
#define EAST 2
#define SOUTHEAST 3
#define SOUTH 4
#define SOUTHWEST 5
#define WEST 6
#define NORTHWEST 7

#pragma DebuggerWindows("DebugStream")

// global variable
int global_orientation;

task main()
{


	//Clears Debug Stream before use
	clearDebugStream();

	//While there are at least 100 bytes available in the buffer
	while(getAvailSpaceInDebugStream() >100)
	{

			int temp = 8*SensorValue(compassWest)+4*SensorValue(compassSouth)+2*SensorValue(compassEast)+SensorValue(compassNorth);

			switch(temp) {
				case 14:
					global_orientation = NORTH;
					break;
				case 13:
					global_orientation = EAST;
					break;
				case 11:
					global_orientation = SOUTH;
					break;
				case 7:
					global_orientation = WEST;
					break;
				case 12:
					global_orientation = NORTHEAST;
					break;
				case 9:
					global_orientation = SOUTHEAST;
					break;
				case 3:
					global_orientation = SOUTHWEST;
					break;
				case 6:
					global_orientation = NORTHWEST;
					break;
				default:
					global_orientation = -1;
			}
		//Write the value of the Sonar sensor to it
		writeDebugStreamLine("The value of digital campass : %f", global_orientation );
		sleep(100);
	}

}
