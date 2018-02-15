package org.usfirst.frc.team5053.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;

public class Elevator implements Subsystem {

	private WPI_TalonSRX m_Talon;
	//private TalonSRXWrapper m_TalonWrapper;
	private DigitalInput m_LimitTop;
	private DigitalInput m_LimitBottom;
	
	PIDController m_PID;
	private final double kp = 0.0;
	private final double ki = 0.0;
	private final double kd = 0.0;
	private double m_PositionTarget;
	
	private final int kPidId = 0;
	private final int kTimeOutMs = 0;
	
	
	
	public Elevator(WPI_TalonSRX speedController)
	{
		m_Talon = speedController;
		
		//m_LimitTop = limitTop;
		//m_LimitBottom = limitBottom;
		m_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPidId,
				kTimeOutMs);
		
		m_PID = new PIDController(kp, ki, kd, (PIDSource) m_Talon, m_Talon);
	}
	public void manualControl(double speed)
	{
		// Runs at half speed for manual control
		m_Talon.set(.5*speed);
	}
	
	public void setPosition(double position)
	{
		m_PID.setSetpoint(position);
		m_PositionTarget = position;
		m_PID.enable();
	}
	
	public void disablePID()
	{
		if(m_PID.isEnabled())
			m_PID.disable();
	}
	
	public boolean isPIDEnabled()
	{
		return m_PID.isEnabled();
	}
	public double getPositionTarget() {
		return m_PositionTarget;
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
