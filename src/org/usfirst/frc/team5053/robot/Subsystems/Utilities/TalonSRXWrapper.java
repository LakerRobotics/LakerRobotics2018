package org.usfirst.frc.team5053.robot.Subsystems.Utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonSRXWrapper implements PIDSource{

	private WPI_TalonSRX m_Talon;
	private PIDSourceType m_SourceType;
	
	public TalonSRXWrapper (WPI_TalonSRX talon) {
		m_Talon = talon;
		m_SourceType = PIDSourceType.kDisplacement;
	}
	@Override
	public void setPIDSourceType(PIDSourceType pidSourceType) {
		// TODO Auto-generated method stub
		m_SourceType = pidSourceType;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		
		return m_SourceType;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		
		if (m_SourceType == PIDSourceType.kRate) {
			return m_Talon.getSelectedSensorVelocity(0);
		} else {
			return m_Talon.getSelectedSensorPosition(0);
		}
	}

}
