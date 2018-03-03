package org.usfirst.frc.team5053.robot.Constants;

public class RobotValuesLilGeek extends RobotValues {
	public String getRobotName() {
		return "lilgeek";
	}
	public int getLeftDrivePWM1()
	{ 
		return 2;
	}
	public int getLeftDrivePWM2()
	{ 
		return 3;
	}
	public int getRightDrivePWM1()
	{ 
		return 0; 
	}
	public int getRightDrivePWM2()
	{ 
		return 1;
	}
	public int getElevatorPWM()
	{
		return 4;
	}
	public int getRightIntakePWM()
	{
		return 5;
	}
	public int getLeftIntakePWM()
	{
		return 6;
	}
	public int getRollerPWM()
	{
		return 7;
	}

	public boolean getLeftDriveInverted()
	{
		return false;
	}
	public boolean getRightDriveInverted()
	{
		return false;
	}
	
	
	public boolean getLeftEncoderInverted()
	{
		return false;
	}
	public boolean getRightEncoderInverted()
	{
		return false;
	}
	
	
	public double getEncoderDistancePerPulse()
	{
		return 6*Math.PI/360;
	}
	
	public int arcadeDriveRotationInverted()
	{
		return 1;
	}
	public int rotationControllerInverted()
	{
		return 1;
	}
}
