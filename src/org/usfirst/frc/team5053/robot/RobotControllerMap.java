package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Maps all of the output controllers on the robot.
 * These include the following:
 * Talons,
 * Victors,
 * Jaguars,
 * Spikes,
 * Solenoids
 */

//Sensors are located and handled by the RobotSensorMap class

public class RobotControllerMap
{
	private final int leftDrive1PWM 	= 0;
	private final int leftDrive2PWM 	= 1;
	private final int rightDrive1PWM 	= 2;
	private final int rightDrive2PWM 	= 3;
	private final int elevatorPWM 		= 4;
	private final int intakeLeftPWM = 5;
	private final int intakeRightPWM = 6;
	private final int catapultSolenoidSlot = 0;
	

	private WPI_TalonSRX m_LeftDrive;
	private WPI_TalonSRX m_LeftDriveSlave;
	private WPI_TalonSRX m_RightDrive;
	private WPI_TalonSRX m_RightDriveSlave;
	private WPI_TalonSRX m_Elevator;
	private WPI_TalonSRX m_Intake1;
	private WPI_TalonSRX m_Intake2;
	
	private Solenoid m_Catapult;
	
	/**
	 * 
	 */
	
	public RobotControllerMap()
	{
		
		m_LeftDrive = new WPI_TalonSRX(leftDrive1PWM);
		m_LeftDriveSlave = new WPI_TalonSRX(leftDrive2PWM);
		m_LeftDriveSlave.follow(m_LeftDrive);
		m_LeftDrive.setInverted(true);
		
		m_RightDrive = new WPI_TalonSRX(rightDrive1PWM);
		m_RightDriveSlave = new WPI_TalonSRX(rightDrive2PWM);
		m_RightDriveSlave.follow(m_RightDrive);
		m_RightDrive.setInverted(true);
		
		m_Elevator = new WPI_TalonSRX(elevatorPWM);
		
		m_Intake1 = new WPI_TalonSRX(intakeLeftPWM);
		m_Intake2 = new WPI_TalonSRX(intakeRightPWM);
		
		m_Catapult = new Solenoid(catapultSolenoidSlot);
	}
	public WPI_TalonSRX getLeftDrive() {
		return m_LeftDrive;
	}
	public WPI_TalonSRX getRightDrive() {
		return m_RightDrive;
	}
	public WPI_TalonSRX getElevator() {
		return m_Elevator;
	}
	public WPI_TalonSRX getLeftIntake() {
		return m_Intake1;
	}
	public WPI_TalonSRX getRightIntake() {
		return m_Intake2;
	}
	public Solenoid getCatapult() {
		return m_Catapult;
	}
}