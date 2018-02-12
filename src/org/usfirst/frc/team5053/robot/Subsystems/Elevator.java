package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;

public class Elevator implements Subsystem {

	private Talon m_Talon;
	private Encoder m_Encoder;
	private DigitalInput m_LimitTop;
	private DigitalInput m_LimitBottom;
	
	PIDController m_PID;
	private final double kp = 0.0;
	private final double ki = 0.0;
	private final double kd = 0.0;
	private double m_PositionTarget;
	
	
	
	public Elevator(Talon speedController, Encoder encoder/*, DigitalInput limitTop, DigitalInput limitBottom*/)
	{
		m_Talon = speedController;
		
		//m_LimitTop = limitTop;
		//m_LimitBottom = limitBottom;
		
		m_PID = new PIDController(kp, ki, kd, m_Encoder, m_Talon);
	}
	public void manualControl(double speed)
	{
		// Runs at half speed for manual control
		m_Talon.setSpeed(.5*speed);
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
