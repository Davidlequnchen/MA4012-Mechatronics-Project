#pragma config(Motor,  port2,           servoTest,     tmotorServoStandard, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// #pragma config(Motor,  port3,           servoCatcher,  tmotorServoStandard, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


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


task main()
{


	servo_to_angle(servoTest, 127,1);		//catcher closes
  motor[servoTest] = 127;

	wait1Msec(1000); //Wait for 1 second
	motor[servoTest] = -127;


}