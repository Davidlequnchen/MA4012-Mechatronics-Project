//Opens Debug Stream Window automatically
#pragma DebuggerWindows("DebugStream")

/*
Display Battery Voltage
ROBOTC on VEX 2.0 Cortex
This program uses the Display functions of ROBOTC on the VEX 2.0 Cortex platform.
It will display the value of the main battery on line 0 and backup battery on line 1.
ROBOT CONFIGURATION
MOTORS & SENSORS:
[I/O Port]					[Name]							[Type]								[Description
UART Port 2					none								VEX LCD								VEX LCD Screen
*/

task main()
{
	string mainBattery;

	//Clears Debug Stream before use
	clearDebugStream();

	//While there are at least 100 bytes available in the buffer
	while(getAvailSpaceInDebugStream() >100)
	{
		writeDebugStreamLine("Battery level: %1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		sleep(1000);
	}
}
