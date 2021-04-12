#pragma config(Sensor, in2,    RightDistanceSensor, sensorAnalog)
#pragma config(Sensor, in3,    LeftDistanceSensor, sensorAnalog)
#pragma config(Sensor, in4,    enemyDistanceSensor, sensorAnalog)
#pragma config(Sensor, in5,    backDistanceSensor, sensorAnalog)
#pragma config(Sensor, in6,    gripperBumper, sensorAnalog)
#pragma config(Sensor, dgtl1,  startSwitch,    sensorTouch)
#pragma config(Sensor, dgtl3,  BackRightSwitch, sensorTouch)
#pragma config(Sensor, dgtl2,  BackLeftSwitch, sensorTouch)
#pragma config(Sensor, dgtl4,  reflectiveFL,   sensorDigitalIn)
#pragma config(Sensor, dgtl5,  reflectiveFR,   sensorDigitalIn)
#pragma config(Sensor, dgtl6,  reflectiveBL,   sensorDigitalIn)
#pragma config(Sensor, dgtl7,  reflectiveBR,   sensorDigitalIn)
#pragma config(Sensor, dgtl8,  compassSupply,  sensorDigitalOut)
#pragma config(Sensor, dgtl9,  compassNorth,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compassEast,    sensorDigitalIn)
#pragma config(Sensor, dgtl11, compassSouth,   sensorDigitalIn)
#pragma config(Sensor, dgtl12, compassWest,    sensorDigitalIn)
#pragma config(Motor,  port2,           servoLeft,     tmotorServoStandard, openLoop)
#pragma config(Motor,  port3,           servoRight,    tmotorServoStandard, openLoop)
#pragma config(Motor,  port4,           leftWheel,     tmotorVex393_MC29, openLoop, reversed, driveLeft)
#pragma config(Motor,  port5,           rightWheel,    tmotorVex393_MC29, openLoop, reversed, driveRight)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

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


//---------------------------function definitions----------------------------------
void differnetial_drive (float leftLevel, float rightLevel);
bool search_ball();
bool move_to_ball();
void release_ball();
void catch_ball();
bool checking_reflective_sensor();
void wait_for_on();
void align_orientation_with_collection();

//---------------------------tasks definitions-------------------------------------
task competition();
task detection();
task read_orientation_campass();
//task detection_others();
//---------------------------global variable definitions---------------------------
bool ballDetected = false;
bool catchmentReady = false;
bool enemyDetected = false;



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

void catch_ball()	//can change to bool when include checking ball
{
    // move forward a little to secure the ball
		clearTimer(T1);
		while(time1[T1]<50)
		{
	     differnetial_drive(-0.7, -0.7);
		}

    motor[servoRight] = 50;       // 45
		motor[servoLeft] = -60;       // -55
		wait1Msec(550);

		// move forward a little to secure the ball
		clearTimer(T1);
		while(time1[T1]<300)
		{
	     differnetial_drive(2, 2);
		}
}

void release_ball()
{
   	align_orientation_with_collection();
		wait1Msec(500);

    clearTimer(T1);
		while(time1[T1]<15000) //
		{

			if (SensorValue[backDistanceSensor] > 450)     //if back distance sensor detects enemy	- NEED TO OPTIMISE
			{
				differnetial_drive(0,0);
				wait1Msec(100);
				differnetial_drive(2,-2);  		// rotate CCW
	  		wait1Msec(200);							// NEED TO OPTIMISE TIME
	  		checking_reflective_sensor();
				differnetial_drive(1.8,1.8);		//drive forward
				wait1Msec(300);
				checking_reflective_sensor();
				align_orientation_with_collection();
				wait1Msec(500);
			}

	    differnetial_drive(-2, -2); // reverse backwards FASTER
	    if (SensorValue(BackLeftSwitch)!=0 && SensorValue(BackRightSwitch)!=0)
	     {
	       // servo release the ball
	        motor[servoRight] = -45;       // -35
					motor[servoLeft] = 90;       // 60
					wait1Msec(1000);
					differnetial_drive(0, 0); //
					wait1Msec(100);
	     }
		}
}


bool checking_reflective_sensor()
{
		if (SensorValue[reflectiveFR] == 0 & SensorValue[reflectiveFL] == 0) //if FRONT back sensors detect, reverse and rotate CCW for 180 degrees
		{
			differnetial_drive(-1.8,-1.8);
	  	wait1Msec(600);	// reverse slightly longer
	  	differnetial_drive(2,-2);  // rotate CCW
	  	wait1Msec(400);
		}

		else if (SensorValue[reflectiveFL] == 0)
	  {
	  	 differnetial_drive(-1.8,-1.8);
	  	 wait1Msec(400);
	  	 differnetial_drive(2,-2);  // rotate CW
	  	 wait1Msec(600);			// does not rotate as much as CCW so longer
		}

		else if (SensorValue[reflectiveFR] == 0)
	  {
	  	 differnetial_drive(-1.8,-1.8);
	  	 wait1Msec(400);
	  	 differnetial_drive(-2,2);  // rotate CCW
	  	 wait1Msec(400);
		}

		else if (SensorValue[reflectiveBR] == 0)
	  {
	     differnetial_drive(1.8,1.8); //drive forward
	     wait1Msec(400);
	  	 differnetial_drive(-2,2);  // rotate CCW
	  	 wait1Msec(400);
	  }

	  else if (SensorValue[reflectiveBL] == 0)
	  {
	     differnetial_drive(1.8,1.8); //drive forward
	     wait1Msec(400);
	  	 differnetial_drive(2,-2);  // rotate CW
	  	 wait1Msec(600);
	  }
	   /*
	  else if (SensorValue[reflectiveFL] == 0 || SensorValue[reflectiveFR] == 0 )
	  {
	  	 differnetial_drive(-2,-2);
	  	 wait1Msec(400);
	  	 differnetial_drive(2,-2);  // rotate CW
	  	 wait1Msec(400);
	  }

	  else if (SensorValue[reflectiveBR] != 0 && SensorValue[reflectiveFR] != 0 && SensorValue[reflectiveFL] != 0 && SensorValue[reflectiveBL] != 0)
		{
				differnetial_drive(1,1);
				wait1Msec(50);
	  }
	  */
	  return false;
}


///------------------------------------------------------

void align_orientation_with_collection()
{
	//rotate to align the orientation with the collection place
  while (true)
  {
		if(global_orientation==EAST||global_orientation==NORTHEAST||global_orientation==SOUTH ||global_orientation==SOUTHEAST)
		{
			while(global_orientation!=NORTH)
			{
				differnetial_drive(-1.5,1.5);//CCW
				wait1Msec(100);
			}
		}
		else if(global_orientation==WEST||global_orientation==NORTHWEST||global_orientation==SOUTHWEST)
		{
			while(global_orientation!=NORTH)
			{
				differnetial_drive(1.5,-1.5);//CW
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

///------------------------------------------------------



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


void start_move()
{
    clearTimer(T1);
		while(time1[T1]<1200)// search for 1.6 seconds
		{
			// if not detected, keep turning CCW within this 1.6 second (90 degree)  ///////////////
      differnetial_drive(2,2);// go straight
		}
}


// return true if ball is detected, otherwise, keep searching
bool search_ball()
{
	while(true) // keep searching
	{
		clearTimer(T1);
		while(time1[T1]<3600) // ROTATE ONE ROUND		//1800
		{
			if(ballDetected) // if either L1 or L3 detect something
			{
				differnetial_drive(0,0); //stop
				return true;
			}
			if (enemyDetected)
	    {
	    	differnetial_drive(0,0);
	    	wait1Msec(500);
	  	}

	  	checking_reflective_sensor();
			// if not detected, keep turning CCW within this 1.6 second (90 degree)  ///////////////
      differnetial_drive(-2, 2); // rotate CCW
		}
		////////////////////////////////////////////////////////////////

		clearTimer(T1);
		while(time1[T1]<800) // search for 1.6 seconds (level 1),,, level2 -- 0.8 sec
		{
			if(ballDetected) // if either L1 or L3 detect something
			{
				differnetial_drive(0,0); //stop
				return true;
			}
			if (enemyDetected)
	    {
	    	differnetial_drive(0,0);
	    	wait1Msec(500);
	  	}

	  	checking_reflective_sensor();
			// if not detected, keep turning CCW within this 1.6 second (90 degree)  ///////////////
      differnetial_drive(1.6, 1.6); // drive forward
		}
		//////////////////////////////////////////////////////////////
		return false;
	}

}


bool move_to_ball()
{
	while (ballDetected)
  {
  	//move towards the ball
		//adjust the speed according to the distance
		if (catchmentReady == true)
	  {
			differnetial_drive(0,0);
			wait1Msec(1000);
			catch_ball();
			if (SensorValue[gripperBumper] == 0)
		  {
		  	return true;
	    }
	    else{
	    	  // servo release the ball
	        motor[servoRight] = -45;       // -35
					motor[servoLeft] = 90;       // 60
					wait1Msec(1000);
	        differnetial_drive(-1,-1);
	        wait1Msec(100);
	        return false;
	    }
	  }
	  else{
	    differnetial_drive(0.85,0.85);
	    wait1Msec(100);
	  }
  }
  return false;
}



void wait_for_on()
{
	while(SensorValue[startSwitch] == 0)
	{
		// while start switch not activated, wait inside the loop
	}

	// start the tasks (multi-tasking parallel thread)
	startTask(competition);
	startTask(read_orientation_campass);
}

/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Tasks (Parallel-runing programs (threads))------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/

task competition()
{
	start_move();         // move half of arena first
	startTask(detection); // start parallel program for ball detection

	while(true)
	{
		if(search_ball()) // check if the ball is detected from either L1 or L3
		{
			if(move_to_ball())
			{
				differnetial_drive(0,0); // stop
				wait1Msec(500);
				release_ball();
			}
		}
	}//end of while true
}



task detection()
{
	// this will always run in the background
	while(true)
	{
		if (SensorValue[enemyDistanceSensor] > 450)     //L1 (left) sensor detects something
		{
			enemyDetected = true;
		}
		else if (SensorValue[enemyDistanceSensor] < 450)     //L1 (left) sensor detects something
		{
			enemyDetected = false;
		}

		if((SensorValue[RightDistanceSensor] > 500 || SensorValue[LeftDistanceSensor] > 300) && enemyDetected == false)
	  {
	  	// 400, 300
			ballDetected = true;
		}

		if((SensorValue[RightDistanceSensor] > 1500 || SensorValue[LeftDistanceSensor] > 1180)  && enemyDetected == false)    //L2 (middle) sensor detects the enemy
		{
			// 1900, 750
			catchmentReady = true;
		}
		else{
			ballDetected = false;
			catchmentReady = false;
		}
	}//end of while

	return;
}//end of task
//-------------------------------------------------------------------------------


task read_orientation_campass()
{
	while(true)
	{
		read_orientation();									//read compass orientation, store it to global_orientation
	}//end of while
	return;
}


// main task
task main()
{
	//startTask(detection); // start parallel program for ball detection

	wait_for_on();
	while(true)
  {
  	wait1Msec(1);
    // Keep the program alive
  }
}//end of main
