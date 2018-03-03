package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Constants.RobotValues;
import org.usfirst.frc.team5053.robot.Constants.RobotValuesLilGeek;
import org.usfirst.frc.team5053.robot.Constants.RobotValuesLisa;

public class RobotConstants 
{
	private static RobotValues robotWeAreRunningOn;
	private static String robotName = "lilgeek"; // Robot name here
	
	static
	{
    	if(robotName.toLowerCase().equals("lisa"))
    	{ 
    		robotWeAreRunningOn = new RobotValuesLisa();
    	}
    	else if(robotName.toLowerCase().equals("lilgeek"))
    	{ 
    		robotWeAreRunningOn = new RobotValuesLilGeek();
    	}
	}

	//???
	public static boolean getRightDriveInverted()
	{
		return robotWeAreRunningOn.getRightDriveInverted();
	}
	public static boolean getLeftDriveInverted()
	{
		return robotWeAreRunningOn.getLeftDriveInverted();
	}
	
	//Drivetrain PWM Slots
	public static int getLeftDrivePWM1()
	{
		return robotWeAreRunningOn.getLeftDrivePWM1();
	}
	public static int getLeftDrivePWM2()
	{
		return robotWeAreRunningOn.getLeftDrivePWM2();
	}
	public static int getRightDrivePWM1()
	{
		return robotWeAreRunningOn.getRightDrivePWM1();
	}
	public static int getRightDrivePWM2()
	{
		return robotWeAreRunningOn.getRightDrivePWM2();
	}
	
	// Subsystems
	public static int getElevatorPWM()
	{
		return robotWeAreRunningOn.getElevatorPWM();
	}
	public static int getRightIntakePWM()
	{
		return robotWeAreRunningOn.getRightIntakePWM();
	}
	public static int getLeftIntakePWM()
	{
		return robotWeAreRunningOn.getLeftIntakePWM();
	}
	public static int getRollerPWM()
	{
		return robotWeAreRunningOn.getRollerPWM();
	}
	
	//Encoder Pulses
	public static double getEncoderDistancePerPulse()
	{
		return robotWeAreRunningOn.getEncoderDistancePerPulse();
	}
	public static boolean getLeftEncoderInverted()
	{
		return robotWeAreRunningOn.getLeftEncoderInverted();
	}
	public static boolean getRightEncoderInverted()
	{
		return robotWeAreRunningOn.getRightEncoderInverted();
	}
	
	//PID values?
	public static double getPidDist_P()
	{
		return robotWeAreRunningOn.getPidDist_P();
	}
	public static  double getPidDist_I()
	{
		return robotWeAreRunningOn.getPidDist_I();
	}
	public static  double getPidDist_D()
	{
		return robotWeAreRunningOn.getPidDist_D();
	}
	public static  double getPidDist_AbsoluteTolerance()
	{
		return robotWeAreRunningOn.getPidDist_AbsoluteTolerance();
	}
	
	public static int getArcadeDriveInverted()
	{
		return robotWeAreRunningOn.arcadeDriveRotationInverted();
	}
	public static int getRotationControllerInverted()
	{
		return robotWeAreRunningOn.rotationControllerInverted();
	}
}