package org.usfirst.frc.team5053.robot.Subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;

public class Catapult implements Subsystem {

	private DoubleSolenoid m_LeftSolenoid;
	private DoubleSolenoid m_RightSolenoid;
	
	public Catapult(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid)
	{
		m_LeftSolenoid = leftSolenoid;
		m_RightSolenoid = rightSolenoid;
	}
	public void Launch() {
		m_LeftSolenoid.set(Value.kForward);
		m_RightSolenoid.set(Value.kForward);
	}
	public void Arm() {
		m_LeftSolenoid.set(Value.kReverse);
		m_RightSolenoid.set(Value.kReverse);
	}
	
	@Override
	public void WriteDashboardData() {
		// TODO Auto-generated method stub
		
	}

}
