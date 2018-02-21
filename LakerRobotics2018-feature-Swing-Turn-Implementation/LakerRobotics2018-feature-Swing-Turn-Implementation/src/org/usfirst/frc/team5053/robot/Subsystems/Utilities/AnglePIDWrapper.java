package org.usfirst.frc.team5053.robot.Subsystems.Utilities;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AnglePIDWrapper implements PIDOutput, PIDSource {
	
	private DriveTrainMotionControl m_DriveTrain;
	
	public AnglePIDWrapper(DriveTrainMotionControl drivetrain)
	{
		m_DriveTrain = drivetrain;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return (m_DriveTrain.GetAngle());
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void pidWrite(double output) {
		// TODO I had to invert this
		m_DriveTrain.setTurn(-output);
	}

}
