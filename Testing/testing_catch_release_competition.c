#pragma config(Sensor, in5,    gripperBumper, sensorAnalog)
#pragma config(Sensor, dgtl1,  masterSwitch,   sensorTouch)
#pragma config(Sensor, dgtl3,  BackRightSwitch, sensorTouch)
#pragma config(Sensor, dgtl2,  BackLeftSwitch, sensorTouch)
#pragma config(Motor,  port2,           servoLeft,     tmotorServoStandard, openLoop)
#pragma config(Motor,  port3,           servoRight,    tmotorServoStandard, openLoop)
#pragma config(Motor,  port4,           leftWheel,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           rightWheel,    tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//-127: fully backward
//MAGNITUDE proportional to POSITION and SPEED



void wait_for_on();


void differnetial_drive(float leftLevel, float rightLevel)
{
	float voltageRight;
	float voltageLeft;

	voltageLeft = leftLevel*30;
	voltageRight = rightLevel*30;

	motor[rightWheel] = -voltageRight;
	motor[leftWheel]  = voltageLeft;
}

/* Catch ball
@param return true if sucessful, false if unsucessful
*/
void catch_ball()	//can change to bool when include checking ball
{
    // move forward a little to secure the ball
		clearTimer(T1);
		while(time1[T1]<50)
		{
	     differnetial_drive(-0.7, -0.7);
		}

    motor[servoRight] = 50;       //
		motor[servoLeft] = -60;       //
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
   	//align_orientation_with_collection();
    wait1Msec(500);

    clearTimer(T1);
		while(time1[T1]<10000) //
		{
	     differnetial_drive(-1, -1); // rotate back
	     if (SensorValue(BackLeftSwitch)!=0 && SensorValue(BackRightSwitch)!=0)
	     {
	       // servo release the ball
	        motor[servoRight] = -45;       // -35
					motor[servoLeft] = 90;       // 60
					wait1Msec(1000);

	        differnetial_drive(0, 0); // rotate back
	     }
		}
}



void wait_for_on()
{
	while(SensorValue(masterSwitch)==0)
	{
		//wait
	}
}

task main()
{
	wait_for_on();
	catch_ball();

	if (SensorValue[gripperBumper] == 0)
	{
			// move forward a little to secure the ball
			clearTimer(T1);
			while(time1[T1]<800)
			{
		     differnetial_drive(1, 1);
			}


			clearTimer(T1);
			while(time1[T1]<10000) // search for 1.6 seconds (level 1),,, level2 -- 0.8 sec
			{
		     differnetial_drive(-2, -2); //  back
		     if (SensorValue(BackLeftSwitch)!=0 && SensorValue(BackRightSwitch)!=0)
		     {
		       release_ball();
		       differnetial_drive(0, 0); //
		     }

			}

  }

  else
	{
	   // servo release the ball
	        motor[servoRight] = -45;       // -35
					motor[servoLeft] = 90;       // 60
					wait1Msec(1000);
			    differnetial_drive(-1,-1);
			    wait1Msec(100);
			    differnetial_drive(0,0);
	}

}
