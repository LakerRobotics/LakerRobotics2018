package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.Sensors.LidarLite;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Maps all of the sensors on the robot.
 * These include but are not limited to the following:
 * Encoders,
 * Potentiometers,
 * Ultrasonics,
 * Limit Switches,
 * Cameras
 */

//Controllers are mapped and handled by the RobotControllerMap class

public class RobotSensorMap
{
	
	private final int leftDriveEncoderADIO = 1; //0
	private final int leftDriveEncoderBDIO = 0; //1
	private final int rightDriveEncoderADIO = 3; //2
	private final int rightDriveEncoderBDIO = 2; //3
	private final int elevatorEncoderADIO = 4;
	private final int elevatorEncoderBDIO = 5;
	private final int elevatorLimitHighDIO = 6;
	private final int elevatorLimitLowDIO = 7;
	
	private Encoder m_LeftDrive;
	private Encoder m_RightDrive;
	private Encoder m_Elevator;
	//private Encoder m_Shooter; if using talon then encoder does not talk to RoboRio so dont define it here, gets configured and defined in TalonSRX motor controllers
	private DigitalInput m_ElevatorLimitHigh;
	private DigitalInput m_ElevatorLimitLow;
	private ADXRS450_Gyro m_Gyro;
	

	public RobotSensorMap()
	{
		m_LeftDrive = new Encoder(leftDriveEncoderADIO, leftDriveEncoderBDIO);
		m_LeftDrive.setDistancePerPulse(6*Math.PI/1024); //Distance in inches
		m_LeftDrive.setReverseDirection(true); // False
		
		m_RightDrive = new Encoder(rightDriveEncoderADIO, rightDriveEncoderBDIO);
		m_RightDrive.setDistancePerPulse(6*Math.PI/1024); //Distance in inches
		//m_RightDrive.setReverseDirection(true);
		
		m_Elevator = new Encoder(elevatorEncoderADIO, elevatorEncoderBDIO);
		m_Elevator.setDistancePerPulse(1/360);
		m_Elevator.setMaxPeriod(1.0);

		m_Gyro = new ADXRS450_Gyro();
		m_ElevatorLimitHigh = new DigitalInput(elevatorLimitHighDIO);
		m_ElevatorLimitLow = new DigitalInput(elevatorLimitLowDIO);
	}
	
	public Encoder getLeftDriveEncoder() 
	{
		return m_LeftDrive;
	}
	public Encoder getRightDriveEncoder() 
	{
		return m_RightDrive;
	}
	public ADXRS450_Gyro getGyro() 
	{
		return m_Gyro;
	}
	public Encoder getElevatorEncoder()
	{
		return m_Elevator;
	}
	public DigitalInput getElevatorLimitHigh()
	{
		return m_ElevatorLimitHigh;
	}
	public DigitalInput getElevatorLimitLow()
	{
		return m_ElevatorLimitLow;
	}
}
