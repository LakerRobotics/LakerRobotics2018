package org.usfirst.frc.team5053.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements Subsystem {

	private TalonSRX m_Talon;
	
	private final double kp = 0.0;
	private final double ki = 0.0;
	private final double kd = 0.0;
	private double m_PositionTarget;
	
	
	
	public Elevator(TalonSRX speedController)
	{
		m_Talon = speedController;
		
		m_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 4000);
	}
	public void manualControl(double speed)
	{
		// Runs at half speed for manual control
		m_Talon.set(ControlMode.PercentOutput, speed);
	}
	
	public void setPosition(double position)
	{
		m_PositionTarget = position;
		m_Talon.set(ControlMode.Position, position);
	}
	
	public void disablePID()
	{
		m_Talon.set(ControlMode.PercentOutput, 0);
	}
	
	public boolean isPIDEnabled()
	{
		return m_Talon.getControlMode().equals(ControlMode.Position);
	}
	public double getPositionTarget() {
		return m_PositionTarget;
	}
	public double getCurrentPosition()
	{
		return m_Talon.getSelectedSensorPosition(0);
	}
	@Override
	public void WriteDashboardData() 
	{
		SmartDashboard.putNumber("Elevator Target", getPositionTarget());
		SmartDashboard.putNumber("Elevator Encoder", getCurrentPosition());
	}

}
