package org.usfirst.frc.team5053.robot.Subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Intake implements Subsystem {

	private Talon m_LeftIntake;
	private Talon m_RightIntake;
	
	private DoubleSolenoid m_Solenoid;
	
	public Intake(Talon leftTalon, Talon rightTalon, DoubleSolenoid solenoid)
	{
		m_LeftIntake = leftTalon;
		m_RightIntake = rightTalon;
		
		m_Solenoid = solenoid;
		
	}
	public void IntakeCube() {
		m_LeftIntake.set(1.0);
		m_RightIntake.set(-1.0);
	}
	public void ReleaseCube() {
		m_LeftIntake.set(-1.0);
		m_RightIntake.set(1.0);
	}
	public void RotateLeft() {
		m_LeftIntake.set(1.0);
		m_RightIntake.set(1.0);
	}
	public void RotateRight() {
		m_LeftIntake.set(-1.0);
		m_RightIntake.set(-1.0);
	}
	public void StopIntake() {
		m_LeftIntake.set(0.0);
		m_RightIntake.set(0.0);
	}
	public void AdjustableSpeed(double speed) {
		m_LeftIntake.set(speed);
		m_RightIntake.set(-speed);
	}
	public void AdjustableSpeedWithTwist(double speed, double twist) {
		m_LeftIntake.set(speed-twist);
		m_RightIntake.set(-speed-twist);
	}
	
	public boolean getSolenoidState()
	{
		Value state = m_Solenoid.get();
		
		return state.equals(Value.kForward);
	}
	public void expandIntake()
	{
		m_Solenoid.set(Value.kForward);
	}
	public void retractIntake()
	{
		m_Solenoid.set(Value.kReverse);
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
