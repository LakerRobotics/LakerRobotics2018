package org.usfirst.frc.team5053.robot.Constants;

/*
 * This is a base orginzational class that provides the pattern for a class that holds all the robot specific setting
 * it is expected there will be a LilGeek, and Lisa implementation of this class
 */
public abstract class RobotValues {

	public String getRobotName() {return "Lilgeek";}
	public boolean getLeftDriveInverted() {return false;}
	public boolean getRightDriveInverted(){return false;}
	
	public int getLeftDrivePWM1() {return 0;}
	public int getLeftDrivePWM2() {return 0;}
	public int getRightDrivePWM1(){return 0;}
	public int getRightDrivePWM2(){return 0;}
	public int getElevatorPWM()   {return 0;}
	public int getRightIntakePWM(){return 0;}
	public int getLeftIntakePWM() {return 0;}
	public int getRollerPWM()	  {return 0;}

	public double getEncoderDistancePerPulse(){return 0;}//converts click to distance in inches
	public boolean getLeftEncoderInverted()	{return false;}
	public boolean getRightEncoderInverted(){return false;}

	// Drive Train motion Control distance
	public double getPidDist_P() {return 0.0;}
	public double getPidDist_I() {return 0.0;}
	public double getPidDist_D() {return 0.0;}
	public double getPidDist_AbsoluteTolerance(){return 0.0;}
		
	public double getMaxRotationSpeed(){return 0.0;}
	public int rotationControllerInverted(){return 0;}
	public int arcadeDriveRotationInverted(){return 0;}
}
