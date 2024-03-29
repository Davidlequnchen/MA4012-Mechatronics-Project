#pragma config(Sensor, in1,    L1,             sensorAnalog)
#pragma config(Sensor, in2,    L2,             sensorAnalog)
#pragma config(Sensor, in3,    L3,             sensorAnalog)
#pragma config(Sensor, in4,    S1,             sensorAnalog)
#pragma config(Sensor, dgtl1,  startSwitch,    sensorTouch)
#pragma config(Sensor, dgtl2,  reflectiveFL,   sensorDigitalIn)
#pragma config(Sensor, dgtl3,  reflectiveFR,   sensorDigitalIn)
#pragma config(Sensor, dgtl4,  reflectiveBL,   sensorDigitalIn)
#pragma config(Sensor, dgtl5,  reflectiveBR,   sensorDigitalIn)
#pragma config(Motor,  port2,           leftWheel,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           rightWheel,    tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//---------------------------function definitions----------------------------------
// void move(int direction, int speedMode);
// void rotate(int direction, int speedMode);
void differnetial_drive (int leftLevel, int rightLevel);
//---------------------------tasks definitions------=------------------------------
// task competition();
// task emergency_stop();



// /*--- linear motion-----
// direction: 1 (forward), -1 (backward)
// speedMode: 0(stop), 1(1/4 speed),2,3(max speed)
// */
// void move(int direction, int speedMode)
// {
// 	int voltageLeft;
// 	int voltageRight;
// 	if(speedMode==1)
// 	{
// 		voltageLeft=30;
// 		voltageRight=30;
// 	}
// 	else if(speedMode==2)
// 	{
// 		voltageLeft=60;
// 		voltageRight=60;
// 	}
// 	else if(speedMode==3)
// 	{
// 		voltageLeft=120;
// 		voltageRight=120;
// 	}
// 	else if(speedMode==0)
// 	{
// 		voltageLeft=0;
// 		voltageRight=0;
// 	}
// 	motor[rightWheel] = -voltageRight*direction;
// 	motor[leftWheel]  = voltageLeft*direction;
// }

// // direction: 1: CCW, -1: CW. speed mode 0123
// void rotate(int direction, int speedMode)
// {
// 	int voltage;
// 	if(speedMode==1)
// 	{
// 		voltage=40;
// 	}
// 	else if(speedMode==2)
// 	{
// 		voltage=60;
// 	}
// 	else if(speedMode==0)
// 	{
// 		voltage=0;
// 	}
// 	motor[rightWheel] = -voltage*direction;
// 	motor[leftWheel]  = -voltage*direction;
// }



/*--- differential drive -----
direction: positive (forward), negative (backward)
speedMode: 0(stop), 1(1/4 speed),2,3,4(max speed), minus/plus sign
Example of function call: differential_drive(1,-1) -- turning CW
*/
void differnetial_drive(int leftLevel, int rightLevel)
{
	int voltageRight;
	int voltageLeft;

	voltageLeft = leftLevel*30; 
	voltageRight = rightLevel*30;
	
	motor[rightWheel] = voltageRight;
	motor[leftWheel]  = -voltageLeft;
}



void wait_for_on()
{
	while(SensorValue[startSwitch] == 0)
	{
		// while start switch not activated, wait inside the loop
	}
	differnetial_drive(1,1); // move straight ahead
	wait1Msec(5000);//wait x second (to be determined)    --- move half of arena
	differnetial_drive(1,-1);
	wait1Msec(1000);

	// button bushed, start the tasks (multi-tasking parallel thread)
	// startTask(competition);
	// startTask(emergency_stop);
}


/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Tasks (Parallel-runing programs (threads))------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
/*
task competition()
{
	startTask(detection);
	startTask(detection_others);

	start_move();

	while(true)
	{

	}//end of while true
}
*/


// main task
task main()
{
	wait_for_on();
	//while(true)
	//{
		// runing until told to stop
	//}
}//end of main
