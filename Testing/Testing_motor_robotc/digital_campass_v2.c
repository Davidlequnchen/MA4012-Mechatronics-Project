#pragma config(Sensor, in5,    gripperBumper, sensorAnalog)
#pragma config(Sensor, dgtl1,  startSwitch,    sensorTouch)
#pragma config(Sensor, dgtl8, compassSupply,  sensorDigitalOut)
#pragma config(Sensor, dgtl12,  compassWest,    sensorDigitalIn)
#pragma config(Sensor, dgtl11,  compassSouth,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compassEast,    sensorDigitalIn)
#pragma config(Sensor, dgtl9, compassNorth,   sensorDigitalIn)
#pragma config(Motor,  port4,           leftWheel,     tmotorVex393_MC29, openLoop, reversed, driveLeft)
#pragma config(Motor,  port5,           rightWheel,    tmotorVex393_MC29, openLoop, reversed, driveRight)

#define NORTH 0
#define NORTHEAST 1
#define EAST 2
#define SOUTHEAST 3
#define SOUTH 4
#define SOUTHWEST 5
#define WEST 6
#define NORTHWEST 7


// global variable
int global_orientation;

// functions and tasks
void differnetial_drive (float leftLevel, float rightLevel);
task read_orientation_campass();



/*--- differential drive -----
direction: 1 (forward), -1 (backward)
speedMode: 0(stop), 1(1/4 speed),2,3,4(max speed), minus/plus sign
*/
void differnetial_drive(float leftLevel, float rightLevel)
{
	float voltageRight;
	float voltageLeft;

	voltageLeft = leftLevel*30;
	voltageRight = rightLevel*30;

	motor[rightWheel] = -voltageRight;
	motor[leftWheel]  = voltageLeft;
}



void align_orientation_with_collection()
{
	//rotate to align the orientation with the collection place
  while (true)
  {
		if(global_orientation==EAST||global_orientation==NORTHEAST||global_orientation==SOUTH ||global_orientation==SOUTHEAST)
		{
			while(global_orientation!=NORTH)
			{
				differnetial_drive(-2,2);//CCW
				wait1Msec(100);
			}
		}
		else if(global_orientation==WEST||global_orientation==NORTHWEST||global_orientation==SOUTHWEST)
		{
			while(global_orientation!=NORTH)
			{
				differnetial_drive(2,-2);//CW
				wait1Msec(100);
			}
		}
		else if (global_orientation==NORTH)
	  {
			differnetial_drive(0,0);//stop
			wait1Msec(100);
			return;
	  }
  }
}




void read_orientation()
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
}


void wait_for_on()
{
	while(SensorValue[startSwitch] == 0)
	{
		// while start switch not activated, wait inside the loop
	}
	startTask(read_orientation_campass);
  align_orientation_with_collection();
  wait1Msec(500);
}


task read_orientation_campass()
{
	while(true)
	{
		read_orientation();									//read compass orientation, store it to global_orientation
	}//end of while
	return;
}




task main()
{
	wait_for_on();
	while(true)
  {
  	wait1Msec(1);
    // Keep the program alive
  }
}
