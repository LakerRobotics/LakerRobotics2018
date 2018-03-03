package org.usfirst.frc.team5053.robot.Constants;

public class RobotValuesLilGeek extends RobotValues {
	
	public int getLeftDrivePWM1()
	{ 
		return 0;
	}
	public int getLeftDrivePWM2()
	{ 
		return 1;
	}
	public int getRightDrivePWM1()
	{ 
		return 2; 
	}
	public int getRightDrivePWM2()
	{ 
		return 3;
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
		return true;
	}
	public boolean getRightDriveInverted()
	{
		return true;
	}
	
	
	public boolean getLeftEncoderInverted()
	{
		return false;
	}
	public boolean getRightEncoderInverted()
	{
		return true;
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
		return -1;
	}
}
