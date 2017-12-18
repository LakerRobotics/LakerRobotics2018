package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * STEAMWORKS 2017
 * Laker Robotics 
 * 
 */

public class Robot extends IterativeRobot
{
	/* Declare any and all variables here
	 *  Do not initialize until robotInit()
	 */
	
	//Robot Map Declaration
	private RobotInterfaceMap m_RobotInterface;
	private RobotControllerMap m_RobotControllers;
	private RobotSensorMap m_RobotSensors;
	
	//Robot Subsystem Declaration
	private DriveTrainMotionControl m_DriveTrain;
	private LidarLite m_Lidar;
	
	//Vision declaration
	
	//Subsystem constants

	//Vision constants
	
	//Autonomous variables
	private int autonomousRoutine;
	private int autonomousCase;
	private int autonomousWait;
	private int allianceSide;
	
	// Diagnostic variables
	private NetworkTable m_NetworkTable;
	private double driveTrainDiagnosticPower;
	private double[] diagnosticLeftRate;
	private double[] diagnosticRightRate;
	private double[] diagnosticPowerSent;
	private int arrIndex;
	
	//Misc variables
	
	
	@Override
    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	m_RobotInterface = new RobotInterfaceMap(JoystickType.XBOX, JoystickType.JOYSTICK);
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDrive(), m_RobotControllers.getRightDrive(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	
    	
    	
    	CameraServer server = CameraServer.getInstance();
    	//server.setQuality(50);
    	server.startAutomaticCapture();
    	
    	// Diagnostic variable initialization
    	m_NetworkTable =  NetworkTable.getTable("SmartDashboard");
    	driveTrainDiagnosticPower = 0;
    	diagnosticLeftRate = new double[202];
    	diagnosticRightRate = new double[202];
    	diagnosticPowerSent = new double[202];
    	arrIndex = 0;
    	
    }

    public void autonomousInit() 
    {
    	 /**
         * This function is called once when autonomous begins
         */
    	
    	autonomousRoutine = (int) SmartDashboard.getNumber("autonRoutine", 0);
    	autonomousCase = 0;
    	autonomousWait = 0;
    	
    	switch(DriverStation.getInstance().getAlliance())
    	{
    	case Red:
    		allianceSide = -1;
    		break;
    	case Blue:
    		allianceSide = 1;
    		break;
		default:
    		allianceSide = 1;
    	}
    	
    }

    public void autonomousPeriodic()
    {
		
    	/**
         * This function is called periodically during autonomous
         */
    	
    	switch(autonomousRoutine)
    	{
    	case 0: // NO AUTON
    		break;
    	case 9: // DEBUG
			diagnosticTest();
			break;
		default: // NO AUTON
			break;
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	autonomousWait++;
    }
    
    public void diagnosticTest()
    {
    	m_DriveTrain.arcadeDrive(driveTrainDiagnosticPower/100, 0);
    	
    	switch(autonomousCase)
    	{
    	case 0:
    		if(autonomousWait >= 10)
        	{
        		if(driveTrainDiagnosticPower <= 99)
        		{
            		diagnosticPowerSent[arrIndex] = driveTrainDiagnosticPower/100;
            		diagnosticLeftRate[arrIndex] = m_DriveTrain.GetLeftSpeed();
            		diagnosticRightRate[arrIndex] = m_DriveTrain.GetRightSpeed();
            		
            		m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower++;
            		autonomousWait = 0;
        		}
        		else
        		{
        			m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
        			m_DriveTrain.arcadeDrive(0, 0);
        			driveTrainDiagnosticPower = 0;
        			autonomousWait = 0;
        			autonomousCase++;
        		}
        	}
    		break;
    	case 1:
    		if(autonomousWait >= 10)
        	{
        		if(driveTrainDiagnosticPower >= -99)
        		{
            		diagnosticPowerSent[arrIndex] = driveTrainDiagnosticPower/100;
            		diagnosticLeftRate[arrIndex] = m_DriveTrain.GetLeftSpeed();
            		diagnosticRightRate[arrIndex] = m_DriveTrain.GetRightSpeed();
            		
            		m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower--;
            		autonomousWait = 0;
        		}
        		else
        		{
        			m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
        			m_DriveTrain.arcadeDrive(0, 0);
        			driveTrainDiagnosticPower = 0;
        			autonomousWait = 0;
        			autonomousCase++;
        		}
        	}
    		break;
		default:
			break;
    	}
    	
    }
    /*public void autonCenter(double visionTurn)
    {
    	switch(autonomousCase)
    	{
    	case 0: // Drive out from wall and engage gear peg
    		System.out.println("Executing Center Autonomous");
    		m_DriveTrain.ResetEncoders();
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.DriveDistance(70, 4.0, 24.0);
    		autonomousCase++;
    		break;
    	case 1: // Disengage gear peg
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			if(autonomousWait >= 100)
				{
					m_DriveTrain.ResetEncoders();
		    		m_DriveTrain.ResetGyro();
		    		m_DriveTrain.DriveDistance(-24, 4, 24);
		    		if(autonomousShoot)
            		{
            			autonomousCase++;
            		}
            		else
            		{
            			autonomousCase = 900;
            		}
    			}
    		}
    		else
    		{
    			autonomousWait = 0;
    		}
    		break;
    	case 2: // Turn to face the boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    			
    			//TODO Blue is very off at 100deg
    			if(allianceSide == -1) // RED
    				m_DriveTrain.TurnToAngle(-98*allianceSide);
    			else // BLUE
    				m_DriveTrain.TurnToAngle(-105.5*allianceSide);
    			
    			autonomousCase++;
    		}
    		break;
    	case 3: // Transition to vision align
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
        		m_DriveTrain.ResetGyro();
        		m_DriveTrain.DriveDistance(102, 8, 24);
        		
        		
    			m_Shooter.SetShooterSetpoint(525);
        		autonomousCase++;
    		}
    		break;
    	case 4: // Vision align to high boiler
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.arcadeDrive(0.6, 0.0);
    			autonomousWait = 0;
    			autonomousCase++;
    			
    		}
    		break;
    	case 5: // Back off the boiler for the diameter of a fuel ball
    		m_DriveTrain.DriveDistance(-5, 4, 1);
    		autonomousCase++;
    	case 6: // Shoot
    		m_DriveTrain.arcadeDrive(0.6, 0.0);
			if (autonomousWait >= 50)
			{
				m_Indexer.SetTalonOutput(INDEXER_SPEED);
				m_DriveTrain.arcadeDrive(0.0, 0.0);
				autonomousCase++;
			}
			break;
    	case 7:
    		break;
		default:
			if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.ResetEncoders();
    			m_DriveTrain.ResetGyro();
    		}
    		break;
    	}
    }
    */
    
    /*public void dumbAutonRoutine()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.ArcadeDrive(0.6, 0);
    		autonomousWait++;
    		if(autonomousWait >= 75)
    		{
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 1:
    		if(autonomousWait >= 100)
    		{
        		autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		m_DriveTrain.ArcadeDrive(-0.6, 0.0);
    		if(autonomousWait >= 25)
    		{
    			m_DriveTrain.ArcadeDrive(0.0, 0.0);
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 3:
    		if(allianceSide == -1) // RED
				m_DriveTrain.TurnToAngle(-100*allianceSide);
			else // BLUE
				m_DriveTrain.TurnToAngle(-102.5*allianceSide);
    		break;
    	case 4:
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			autonomousCase++;
    			autonomousWait = 0;
    		}
    		break;
    	case 5:
    		m_DriveTrain.ArcadeDrive(0.6, 0);
    		
    		if(autonomousWait >= 150)
    		{
    			m_DriveTrain.ArcadeDrive(0.0, 0.0);
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    	case 6:
    		if(autonomousWait == 0)
    		{
        		m_Shooter.EnablePID();
    			m_Shooter.SetShooterSetpoint(530);
    		}
    		
    		if(autonomousWait >= 50)
    		{
    			m_Indexer.SetTalonOutput(INDEXER_SPEED);
    		}
    		break;
    	}
    }
    */
    
    
    
    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	
    	if(autonomousCase != 0)
    	{
    		autonomousCase = 0;
    		m_DriveTrain.DisablePIDControl();
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	
    	arcadeDrive();
    	m_DriveTrain.WriteDashboardData();
    	
    	//Shooter methods
    	
    	//Other
    	
    	//Misc variable updates
    	
    }

    public void testPeriodic()
    {  
        /**
         * This function is called periodically during test mode
         */
    }
    
    //Drivetrain methods
    public void arcadeDrive()
    {
    	if(m_RobotInterface.GetDriverLeftTrigger())
    	{
    	 	m_DriveTrain.ArcadeDrive(m_RobotInterface.GetDriverLeftY()*0.7, m_RobotInterface.GetDriverRightX()*0.7);
    	}
    	else
    	{
    	 	m_DriveTrain.ArcadeDrive(m_RobotInterface.GetDriverLeftY(), m_RobotInterface.GetDriverRightX());
    	}
   }
    
    
    public void GetDashboardData()
    {
    	//shooterRPM = SmartDashboard.getNumber("shooterRPM", DEFAULT_SHOOTER_RATE);
    	//shooterRPMBF = SmartDashboard.getNumber("shooterRPMBF", DEFAULT_SHOOTER_RATE);
    	//Use this to retrieve values from the Driver Station
    	//e.g Which autonomous to use or processed image information.
    }
    public void WriteDashboardData()
    {
    	m_DriveTrain.WriteDashboardData();
    	SmartDashboard.putNumber("lidar", m_Lidar.getDistanceFt());
    }
}