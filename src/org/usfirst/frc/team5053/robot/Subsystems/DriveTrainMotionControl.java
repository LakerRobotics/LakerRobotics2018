package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import org.usfirst.frc.team5053.robot.Subsystems.Utilities.MotionController;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * 
 *
 */

public class DriveTrainMotionControl extends DifferentialDrive implements Subsystem
{
	/**
	 * Hello There! : I'm the base constructor.
	 */
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private MotionController m_MotionController;
	
	private ADXRS450_Gyro m_Gyro;
	
	public boolean isPIDRunning = false;
	
	
	public DriveTrainMotionControl(SpeedControllerGroup leftMotorGroup, SpeedControllerGroup rightMotorGroup, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro Gyro)
	{
		super(leftMotorGroup, rightMotorGroup);
		
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_Gyro = Gyro;
		
		m_MotionController = new MotionController(this, (PIDSource) m_RightEncoder, (PIDSource) m_Gyro);
		
	}
	public void DriveDistance(double distance, double maxspeed, double ramp)
	{
		if(!isPIDRunning)
		{
			isPIDRunning = 	m_MotionController.ExecuteStraightMotion(distance, maxspeed, ramp);
		}
		
	}
	public void TurnToAngle(double turnAngle)
	{
		if(!isPIDRunning)
		{
			isPIDRunning = m_MotionController.ExecuteTurnMotion(turnAngle);
		}
		
	}
	public void DriveInArc(double distance, double maxspeed, double ramp, double radius)
	{
		if(!isPIDRunning)
		{
			isPIDRunning = m_MotionController.ExecuteArcMotion(distance, maxspeed, ramp, radius);
		}
	}
	public boolean isStraightPIDFinished()
	{
		if(m_MotionController.isStraightMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public boolean isTurnPIDFinished() 
	{
		if(m_MotionController.isTurnMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public boolean isArcPIDFinished()
	{
		if(m_MotionController.isArcMotionFinished())
		{
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	public void DisablePIDControl()
	{
		m_MotionController.DisablePIDControls();
	}
	public double GetRightDistance()
	{
		return m_RightEncoder.getDistance();
	}
	public double GetRightSpeed()
	{
		return m_RightEncoder.getRate();
	}
	public double GetLeftDistance()
	{
		return m_LeftEncoder.getDistance();
	}
	public double GetLeftSpeed()
	{
		return m_LeftEncoder.getRate();
	}
	public void ResetEncoders()
	{
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}
	public void ResetGyro() 
	{
		m_Gyro.reset();
	}
	public double GetAverageSpeed()
	{
		return -(-GetLeftSpeed() + GetRightSpeed())/2;
	}
	public double GetAverageDistance()
	{
		return -(-GetLeftDistance() + GetRightDistance())/2;
	}
	public void ArcadeDrive(double speed, double angle)
	{
		this.arcadeDrive(speed, angle);
	}
	public double GetAngle()
	{
		return m_Gyro.getAngle();
	}
	public double getAngularVelocity()
	{
		return m_Gyro.getRate();
	}
	
	public HashMap<String, Double> GetDashboardData() 
	{
		return null;
		// TODO Auto-generated method stub
		
	}
	
	public void WriteDashboardData() 
	{
		SmartDashboard.putNumber("Gyro Angle", m_Gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", m_Gyro.getRate());
		SmartDashboard.putNumber("LeftDriveEncoder Rate", m_LeftEncoder.getRate());
		
		// Do not change these names they are used for the DS Dashboard
		SmartDashboard.putNumber("leftDriveEncoder", m_LeftEncoder.getDistance());
		SmartDashboard.putNumber("rightDriveEncoder", m_RightEncoder.getDistance());
	}
	@Override
	public boolean isClosedLoopControl() {
		// TODO Auto-generated method stub
		return isPIDRunning;
	}
	@Override
	public boolean enableClosedLoopControl(double target, double speed, double ramp) {
		// TODO Auto-generated method stub
		return false;
	}
	@Override
	public boolean disableClosedLoopControl() {
		// TODO Auto-generated method stub
		return false;
	}
	@Override
	public boolean isOnTarget() {
		// TODO Auto-generated method stub
		return false;
	}

}
