#pragma config(Motor,  port2,           servoSwinger,  tmotorServoStandard, openLoop, driveLeft)
#pragma config(Motor,  port3,           servoCatcher,  tmotorServoStandard, openLoop, driveRight)
#pragma config(Sensor, dgtl2,  leftBumper,         sensorTouch)
#pragma config(Sensor, dgtl1,  rightBumper,         sensorTouch)
#pragma config(Sensor, dgtl3,  rearBumper,         sensorTouch)
#pragma config(Sensor, port8,  masterSwitch,         sensorAnalog) //TODO change pins
#pragma config(Sensor, in2,    B,              sensorAnalog)
#pragma config(Sensor, in3,    C,              sensorAnalog)
#pragma config(Sensor, in4,    A,              sensorAnalog)
#pragma config(Sensor, in5,    CheckBallPosition, sensorAnalog)
#pragma config(Motor,  port10,           rightWheel,    tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port1,           leftWheel,     tmotorNormal, openLoop)
#pragma config(Sensor, dgtl6,  reflectiveFL,   sensorDigitalIn)
#pragma config(Sensor, dgtl7,  reflectiveFR,   sensorDigitalIn)
#pragma config(Sensor, dgtl5,  reflectiveBL,   sensorDigitalIn)
#pragma config(Sensor, dgtl4,  reflectiveBR,   sensorDigitalIn)
#pragma config(Sensor, dgtl8, compassSupply,  sensorDigitalOut)
#pragma config(Sensor, dgtl12,  compassWest,    sensorDigitalIn)
#pragma config(Sensor, dgtl11,  compassSouth,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compassEast,    sensorDigitalIn)
#pragma config(Sensor, dgtl9, compassNorth,   sensorDigitalIn)

#define ANGLE_CATCHER_OPEN -126
#define ANGLE_CATCHER_CLOSE 60
#define ANGLE_SWINGER_INIT -50
#define ANGLE_SWINGER_SECURE 15
#define ANGLE_SWINGER_RELEASE 115

#define DELAY_SWINGER 8
#define DELAY_CATCHER 1

#define NORTH 0
#define NORTHEAST 1
#define EAST 2
#define SOUTHEAST 3
#define SOUTH 4
#define SOUTHWEST 5
#define WEST 6
#define NORTHWEST 7

#define STATE_MOVE_TO_BALL 1
#define STATE_SEARCH_BALL 2
#define STATE_GO_TO_COLLECTION_PLACE 3

/*-----------------------------------global variables------------------------------------*/
bool ballClose=false;				//specify whether ball is close enough to be grabbed or not
bool ballDetected=false;
bool robotDetected=false;
int batLevel;
float distanceB;
bool ballCollected=false; 				//specify whether ball is already scoped or not for debugging only
bool rearBumperPressed=false;
bool leftBumperPressed=false;
bool rightBumperPressed=false;
int global_orientation;
int codeState=0; 								//variable to tell where the code line is at

bool already_in_collection_place=false;
int debugVar=0;
void align_orientation_with_collection();
void start_move();
/*-----------------------------------end of global variables------------------------------------*/

void read_orientation();
bool is_ball_scoped();
void servo_to_angle(int port, int angle_destination, int delay_speed);
bool catch_ball();
void move(int direction, int speedMode);
void rotate(int direction, int speedMode);
void reset_servo();
bool go_to_collection_place();
bool move_to_ball();
bool search_ball();
task competition();
task emergency_stop();
void wait_for_on();
bool line_detection();
/*-----------------------------------end of global variables----------------------------*/

/* Use the upper sensor to detect whether ball has been successfully scoped on the grabber or not
@param return true if yes, false if no
*/
bool is_ball_scoped()
{
	if(SensorValue[CheckBallPosition]>1200){
		ballCollected = true;
		return true;
	}
	else{
		ballCollected=false;
		return false;
	}
}

void servo_to_angle(int port, int angle_destination, int delay_speed)
{
	while(motor[port]>angle_destination)
	{
		motor[port]=motor[port]-1;
		wait1Msec(delay_speed);
	}
	while(motor[port]<angle_destination)
	{
		motor[port]=motor[port]+1;
		wait1Msec(delay_speed);
	}
}

/* Catch ball
@param return true if sucessful, false if unsucessful
*/
bool catch_ball()
{
	servo_to_angle(servoCatcher,ANGLE_CATCHER_CLOSE,DELAY_CATCHER);		//catcher closes
	wait1Msec(500);																										//wait
	servo_to_angle(servoSwinger,ANGLE_SWINGER_SECURE,DELAY_SWINGER);	//Swinger scopes ball
	wait1Msec(700);																										//wait

	if (is_ball_scoped())																									//detect if ball is scoped
	{
		servo_to_angle(servoSwinger,ANGLE_SWINGER_SECURE+30,DELAY_SWINGER);//move the swinger abit higher to further secure the ball
		wait1Msec(100);
		servo_to_angle(servoCatcher,ANGLE_CATCHER_OPEN,DELAY_CATCHER);		//catcher goes back to initial position
		return true;
	}
	else
	{
		//servo initialisation
		servo_to_angle(servoSwinger,ANGLE_SWINGER_INIT,DELAY_SWINGER);
		servo_to_angle(servoCatcher,ANGLE_CATCHER_OPEN,DELAY_SWINGER);
		return false;
	}
	return false;
}

bool line_detection()
{
	if(codeState==STATE_GO_TO_COLLECTION_PLACE)
	{
		//do some action about the line
		if (SensorValue[reflectiveFL]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(1000);
			rotate(-1,1);
			wait1Msec(1000);
			move(1,1);
			wait1Msec(1000);
			align_orientation_with_collection();
		}
		else if(SensorValue[reflectiveFR]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(1000);
			rotate(1,1);
			wait1Msec(1000);
			move(1,1);
			wait1Msec(1000);
			align_orientation_with_collection();
		}
		else if(SensorValue[reflectiveBL]==0)
		{
			move(1,0);

			clearTimer(T2);
			while(time1(T2)<700)
			{
				move(-1,1);
				if(rearBumperPressed)
				{
					clearTimer(T3);
					while(time1[T3] < 1000)
					{
						if(SensorValue[reflectiveBR]==1 && SensorValue[reflectiveBL]==1)
						{
							move(1,1);
						}
						else
						{
							move(1,0);
							already_in_collection_place=true;
							return true;
						}
					}//end of inner while
					align_orientation_with_collection();
					already_in_collection_place=false;
					return false;
				}//end of if bumper detected
			}//end of outer while

			move(1,1);
			wait1Msec(1000);
			rotate(-1,1);
			wait1Msec(1000);
			move(1,1);
			wait1Msec(1000);
			align_orientation_with_collection();
			already_in_collection_place=false;
			return false;
		}
		else if(SensorValue[reflectiveBR]==0)
		{
			move(1,0);

			clearTimer(T2);
			while(time1(T2)<700)
			{
				move(-1,1);
				if(rearBumperPressed)
				{
					clearTimer(T3);
					while(time1[T3] < 1000)
					{
						if(SensorValue[reflectiveBR]==1 && SensorValue[reflectiveBL]==1)
						{
							move(1,1);
						}
						else
						{
							move(1,0);
							already_in_collection_place=true;
							return true;
						}
					}//end of inner while
					align_orientation_with_collection();
					already_in_collection_place=false;
					return false;
				}//end of if bumper detected
			}//end of outer while

			move(1,1);
			wait1Msec(1000);
			rotate(1,1);
			wait1Msec(1000);
			move(1,1);
			wait1Msec(1000);
			align_orientation_with_collection();
			already_in_collection_place=false;
			return false;
		}
	}
		//do some action about the line
		else if(SensorValue[reflectiveFL]==0 && SensorValue[reflectiveFR]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(1000);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		else if (SensorValue[reflectiveFL]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(1000);
			rotate(-1,1);		//CW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveFR]==0)
		{
			move(1,0);
			move(-1,1);
			wait1Msec(1000);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveBL]==0 && SensorValue[reflectiveBR]==0)
		{
			move(1,0);
			move(1,1);
			wait1Msec(1000);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveBL]==0)
		{
			move(1,0);
			move(1,1);
			wait1Msec(1000);
			rotate(-1,1);		//CW
			wait1Msec(800);
		}
		else if(SensorValue[reflectiveBR]==0)
		{
			move(1,1);
			wait1Msec(1000);
			rotate(1,1);		//CCW
			wait1Msec(800);
		}
		return false;
}

task detection_others()
{
	while(true)
	{
		read_orientation();									//read compass orientation, store it to global_orientation

		/*--------------------------Read bumpers---------------*/
		if (SensorValue(leftBumper) == 1)
		{
			leftBumperPressed= true;
		}
		else if(SensorValue(leftBumper)==0)
		{
			leftBumperPressed= false;
		}
		if (SensorValue(rightBumper) == 1)
		{
			rightBumperPressed= true;
		}
		else if(SensorValue(rightBumper)==0)
		{
			rightBumperPressed=false;
		}
		if(SensorValue(rearBumper)==1)
		{
			rearBumperPressed=true;
		}
		else if(SensorValue(rearBumper)==0)
		{
			rearBumperPressed=false;
		}
		/*--------------------------end of Read bumpers---------------*/
	}//end of while
}

void move(int direction, int speedMode)
{
	int voltageLeft;
	int voltageRight;
	if(speedMode==1)
	{
		voltageLeft=30;
		voltageRight=30;
	}
	else if(speedMode==2)
	{
		voltageLeft=60;
		voltageRight=60;
	}
	else if(speedMode==3)
	{
		voltageLeft=120;
		voltageRight=120;
	}
	else if(speedMode==0)
	{
		voltageLeft=0;
		voltageRight=0;
	}
	motor[rightWheel] = voltageRight*direction;
	motor[leftWheel]  = voltageLeft*direction;

}

void rotate(int direction, int speedMode)
{
	int voltage;
	if(speedMode==1)
	{
		voltage=40;
	}
	else if(speedMode==2)
	{
		voltage=60;
	}
	else if(speedMode==0)
	{
		voltage=0;
	}
	motor[rightWheel] = voltage*direction;
	motor[leftWheel]  = -voltage*direction;
}

task detection()
{
	while(true)
	{
		if(500<SensorValue[B] && SensorValue[B]< 3000) 		//lower sensor detects something
		{
			if (SensorValue[C] >700) 														//upper sensor detects something TODO change value
			{
				if(SensorValue[B]>SensorValue[C]+300)									//if the B>C by ~6cm, then there is ball too
				{
					ballDetected=true;
					robotDetected=true;
				}
				else																									//otherwise
				{
					ballDetected=false;
					robotDetected=true;
				}
			}
			else																								//upper sensor does not detect anything
			{
				robotDetected = false;
				ballDetected=true;
			}
		}
		else
		{
			ballDetected=false;
			robotDetected=false;
		}
		//If ball is detected, Check if the ball is close enough
		if(ballDetected)
		{
			distanceB = (10000.0/SensorValue[B]-0.6685)/0.4255;

			if(SensorValue[A] > 910 && SensorValue[A] <1500) 		//side sensor detects within the range acceptable for grabber
			{
				ballClose =true;	//ball is close enough for the grabber
			}
			else
			{
				ballClose = false; //not close enough
			}
		}
	}//end of while
}//end of task


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


void reset_servo()
{
	servo_to_angle(servoCatcher,0,DELAY_CATCHER);
	servo_to_angle(servoSwinger,0,DELAY_SWINGER);
}

/*
@param return true, if ball is right in front of the ball
*/
bool move_to_ball()
{
	int aaa=0;
	codeState=STATE_MOVE_TO_BALL;							//indicate the code is now at move_to_ball function
	while(ballDetected)			//ball detected
	{
		//move towards the ball
		//adjust the speed according to the distance
		if(ballClose)
		{
			debugVar=100;
			move(1,0);											//stop
			wait1Msec(500);
			return true;
		}
		else if(SensorValue[B]>1100)
		{
			//no need line detection
			move(1,1);
			if(aaa ==0)
			{
				clearTimer(T4);
			}
			aaa=1;
			if(time1(T4)>3000)
			{
				if(global_orientation==5 ||global_orientation==6 )
				{
					move(-1,1);
					wait1Msec(1000);
					align_orientation_with_collection();
					start_move();
				}

			}

		}
		else if(SensorValue[B]>800) //TODO change me
		{
			line_detection(); //AAA
			move(1,2);			//lower down the speed
			//align TODO
		}
		else
		{
			line_detection(); //AAA
			move(1,2);
		}
		/*if(sensorValue(A)> 0 && sensorValue(A)< 100)//TODO put number
		{
		if(ballClose)
		{
		//grab
		//catch_ball(); //CHANGEME
		move(1,0);
		//TODO
		}
		else if(!ballClose)
		{
		//align TODO
		}
		}*/

	}//end of while

	/*--if suddenly ball is not detected, rotate abit to left and right*/
	clearTimer(T4);

	int lastRotate=0;
	clearTimer(T1);
	while(time1[T1]<700 && !ballDetected)//TODO change the value .,tune!
	{
		line_detection(); //AAA
		rotate(1,1);
		lastRotate=1;
	}
	if(ballDetected&& lastRotate==1)
	{
		clearTimer(T1);
		while(time1[T1]<50)//TODO change the value .,tune!
		{
			rotate(1,1);
			if(!ballDetected)
			{
				//go to search_ball TODO
				move(1,0);//CHANGEME TODO
				debugVar=3;
			}
		}
	}
	clearTimer(T1);
	while(time1[T1]<2*700 && !ballDetected)
	{
		line_detection(); //AAA
		rotate(-1,1)
		lastRotate=2;
	}
	if(ballDetected&& lastRotate==2)
	{
		clearTimer(T1);
		while(time1[T1]<50)//TODO change the value .,tune!
		{
			rotate(-1,1);
			if(!ballDetected)
			{
				//go to search_ball TODO
				move(1,0);//CHANGEME TODO
				debugVar=4;
			}
		}
	}

	if(ballDetected)
	{
		move_to_ball();
	}
	else if(!ballDetected)
	{
		return false;
	}
	return false;
}//end of move_to_ball

void align_orientation_with_collection()
{
	//rotate to align the orientation with the collection place
	if(global_orientation==0||global_orientation==1||global_orientation==7||global_orientation==6)
	{
		while(global_orientation!=2)
		{
			rotate(-1,1);//CW
		}
	}
	else if(global_orientation==2||global_orientation==3||global_orientation==4||global_orientation==5)
	{
		while(global_orientation!=1)
		{
			rotate(1,1);//CCW
		}
	}
	move(1,0);
}
/*
*/
bool go_to_collection_place()
{
	already_in_collection_place=false;
	codeState=STATE_GO_TO_COLLECTION_PLACE;			//indicate the code is now at go_to_collection_place function

	align_orientation_with_collection();
	wait1Msec(500);
	//move backward until bumper detected
	while(!already_in_collection_place) //move backward
	{
		line_detection();
		if(already_in_collection_place)
		{
			return true;
		}
		move(-1,3);
	}
	return false;
}

/*
	Search for ball
	@param stop and return true when ball is found
*/
bool search_ball()
{
	codeState=STATE_SEARCH_BALL; //indicate the code is now at search_ball function

	while(true)
	{
		debugVar=25;
		clearTimer(T1);
		while(time1[T1]<600)	//change the value 3000 TODO
		{
			//line_detection();//AAA
			if(ballDetected)
			{
				move(1,0);				//stop
				return true;
			}
			rotate(1,2);				//rotate CCW
		}

		clearTimer(T1);
		while(time1[T1]<2*700)	//change the value 3000 TODO
		{
			//line_detection();//AAA
			if(ballDetected)
			{
				move(1,0);				//stop
				return true;
			}
			rotate(-1,2);				//rotate CW
		}
		debugVar=-500;

		clearTimer(T1);
		while(time1[T1]<600)	//change the value 3000 TODO
		{
			//line_detection();//AAA
			if(ballDetected)
			{
				move(1,0);				//stop
				return true;
			}
			rotate(1,2);				//rotate CCW
		}
		move(1,0);

		//if ball is not detected during rotation
		clearTimer(T1);
		while(time1[T1]<1000)	//change tehe value 2000 TODO
		{
			move(1,2);
			line_detection();//AAA
		}
		return false;//note
	}

	return false;
}

/*
Initial action, move half of the arena
*/
void start_move()
{
	clearTimer(T1);
	while(time1(T1)<2000)//TODO tune me
	{
		move(1,3);
	}
}





/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
task competition()
{
	startTask(detection);
	startTask(detection_others);

	start_move();	//TODO uncomment me

	while(true)
	{
		if(search_ball())
		{
			debugVar=400;
			if(move_to_ball())
			{
				debugVar=55;
				if(catch_ball())
				{
					while(!go_to_collection_place())
					{
					}
					servo_to_angle(servoSwinger,ANGLE_SWINGER_RELEASE,DELAY_SWINGER);
					wait1Msec(1500);
					servo_to_angle(servoSwinger,ANGLE_SWINGER_INIT,DELAY_SWINGER);
					servo_to_angle(servoCatcher,ANGLE_CATCHER_OPEN,DELAY_SWINGER);
					start_move();
				}
			}
		}
	}//end of while true
}

task emergency_stop()
{
	while(true)
	{
		if(SensorValue[masterSwitch]==0)
		{
			stopTask(competition);
			move(1,0);
			servo_to_angle(servoSwinger,ANGLE_SWINGER_INIT,DELAY_SWINGER);
			servo_to_angle(servoCatcher,ANGLE_CATCHER_OPEN,DELAY_SWINGER);
			break;
		}
	}
	wait_for_on();
}

void wait_for_on()
{
	while(SensorValue(masterSwitch)==0)
	{
		//wait
	}
	startTask(competition);
	startTask(emergency_stop);
}

task main()
{
	servo_to_angle(servoSwinger,ANGLE_SWINGER_INIT,DELAY_SWINGER);
	servo_to_angle(servoCatcher,ANGLE_CATCHER_OPEN,DELAY_SWINGER);

	wait_for_on();
	while(true)
	{
	}
}//end of main
