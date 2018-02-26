package org.usfirst.frc.team5053.robot.Constants;

public class RobotValuesLisa extends RobotValues 
{
	public int getLeftDrivePWM1()
	{
		return 7;
	}
	public int getLeftDrivePWM2()
	{
		return 8;
	}
	public int getRightDrivePWM1()
	{
		return 3;
	}
	public int getRightDrivePWM2()
	{
		return 4;
	}
	public int getElevatorPWM()
	{
		return 2;
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
		return true;
	}
	public boolean getRightEncoderInverted()
	{
		return false;
	}
	
	
	public double getEncoderDistancePerPulse()
	{
		return 6*Math.PI/1024;
	}
}
