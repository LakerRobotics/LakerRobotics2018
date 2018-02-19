package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.JoystickType;
import org.usfirst.frc.team5053.robot.Sensors.LidarLite;
import org.usfirst.frc.team5053.robot.Subsystems.Catapult;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrainMotionControl;
import org.usfirst.frc.team5053.robot.Subsystems.Elevator;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * Power Up 2018
 * Laker Robotics 
 * 
 * 
 * Driver Controls
 * Left Stick - Pitch
 * Right Stick - Yaw
 * Left Trigger - 70% Speed
 * Right Trigger - Fire Catapult (If armed)
 * Left Bumper - 100% Speed
 * Right Bumper -
 * Select -
 * Start -
 * A -
 * B -
 * X -
 * Y - 
 * 
 * 
 * Operator Controls
 * Joystick Y - Manual Elevator Height
 * Trigger	- Retract Intake Solenoid
 * Button 3 - Intake Cube
 * Button 4 - Release Cube
 * Button 5 - Rotate Cube Left
 * Button 6 - Rotate Cube Right
 * ?? - Elevator to Floor Pickup
 * ?? - Elevator to Transfer
 * ?? - Elevator to Switch High
 * ?? - Elevator to Switch Low
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
	private Elevator m_Elevator;
	private Intake m_Intake;
	private Catapult m_ThePult;
	private LidarLite m_Lidar;
	private Compressor m_Compressor;
	
	//Vision declaration
	
	//Subsystem constants

	//Vision constants
	
	//Autonomous variables
	private int autonomousRoutine;
	private int autonomousCase;
	private int autonomousWait;
	
	private String autonRoutine;	
	private boolean secondPart;
	private String matchData;
	private int switchTurn;
	private int scaleTurn;
	private char switchChar;
	private char scaleChar;
	
	// Diagnostic variables
	//private NetworkTable m_NetworkTable;
	private double driveTrainDiagnosticPower;
	private double[] diagnosticLeftRate;
	private double[] diagnosticRightRate;
	private double[] diagnosticPowerSent;
	private int arrIndex;
	
	
	
	//Misc variables
	// TODO MUST BE INVERTED the SRX encoder is wired incorrectly and we can't invert only the encoder 
	private final double kFloor = 0.0;
	private final double kTransfer = -13290.0;
	private final double kLow = 0.0;
	private final double kHigh = -19569.0;
	
	private double m_driveSpeed = 1.0;
	
	private int m_catapultDelay = 0;
	
	private final int SWITCH_CATAPULT_DELAY = 5;
	private int m_shortCatapultDelay = 0;
	private boolean isShotFinished = true;
	
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
    	m_DriveTrain = new DriveTrainMotionControl(m_RobotControllers.getLeftDriveGroup(), m_RobotControllers.getRightDriveGroup(), m_RobotSensors.getLeftDriveEncoder(), m_RobotSensors.getRightDriveEncoder(), m_RobotSensors.getGyro());
    	m_Elevator = new Elevator(m_RobotControllers.getElevator());
    	m_Intake = new Intake(m_RobotControllers.getLeftIntake(), m_RobotControllers.getRightIntake(), m_RobotControllers.getIntakeSolenoid());
    	m_ThePult = new Catapult(m_RobotControllers.getCatapultLeft(), m_RobotControllers.getCatapultRight());
    	// Scaler
    	
    	CameraServer server = CameraServer.getInstance();
    	server.startAutomaticCapture();
    	
    	m_Compressor = new Compressor(0);

    	//m_Compressor.start();
    	m_Compressor.setClosedLoopControl(true);
    	
    	// Diagnostic variable initialization
    	//m_NetworkTable =  NetworkTable.getTable("SmartDashboard");
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
    	
    	// Initialize autonomous variables
    	autonomousCase = 0;
    	autonomousWait = 0; // 20 loops per second
    	
    	// Get information about which autonomous routine to run
    	// TODO Make sure this is defaulted to the correct value when put to production
    	autonRoutine = "test"; //SmartDashboard.getString("Auton Selection", "center scale");		// Start position of the robot from our side of the field
    	secondPart = SmartDashboard.getBoolean("Second Step", false);			// Second part of auton routine
    	matchData = DriverStation.getInstance().getGameSpecificMessage(); 		// Field orientation
    	
    	// Parse matchData  for the switch and scale position in order to determine which way to turn later
    	switchChar = matchData.charAt(0);
    	scaleChar = matchData.charAt(1);
    	
    	if(switchChar == 'R')
    		switchTurn = 1; // Determines the switch plate we aim at where positive turns right(clockwise)
    	else
    		switchTurn = -1; // And negative turns left (counterclockwise) 
    	
    	if(scaleChar == 'R')
    		scaleTurn = -1; // Final turn is always Counter clockwise when the scale is on the right side
    	else
    		scaleTurn = 1; // And vice versa
    	
    	m_DriveTrain.ResetGyro();
    	m_DriveTrain.ResetEncoders();
    	m_Elevator.resetEncoder();
    }

    public void autonomousPeriodic()
    {
		
    	/**
         * This function is called periodically during autonomous
         */
    	switch(autonRoutine.toLowerCase())
    	{
    	case "none": // NO AUTON
    		break;
    	case "switch": // CENTER AUTON SWITCH FIRST
    		switchCenter();
			break;
    	case "left scale": // LEFT AUTON SCALE FIRST
    		//scaleLeftRight();
    		worstCaseScenarioScaleLeftRightTest();
    		break;
    	case "center scale": // CENTER AUTON SCALE
    		scaleCenter();
    		break;
    	case "right scale": // RIGHT AUTON SCALE FIRST
    		//scaleLeftRight();
    		worstCaseScenarioScaleLeftRightTest();
    		break;
    	case "diagnostic": // DEBUG AUTO
			diagnosticTest();
			break;
    	case "test":
    		switchShotTest();
    		//swingTest();
    		//straightTest();
    		//controlledAngleTest();
    		//turnTest();
    		break;
		default: // NO AUTON
			break;
    	}
    	
    	GetDashboardData();
    	WriteDashboardData();
    	
    	autonomousWait++;
    }
    
    public void controlledAngleTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.DriveControlledAngle(120, 5, 20, 0);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
    }
    public void turnTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.TurnToAngle(90);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isTurnPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.ArcadeDrive(0, 0);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
    }
    public void straightTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		m_DriveTrain.DriveDistance(120, 8, 20);
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			m_DriveTrain.ArcadeDrive(0, 0);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
    } 
    public void swingTest()
    {
    	switch(autonomousCase)
    	{
    	case 0:
    		SmartDashboard.putBoolean("Swing test complete", false);
    		m_DriveTrain.ResetGyro();
    		m_DriveTrain.SetSwingParameters(-45, true);
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1:
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DriveControlledAngle(2*12, 5, 20, -45);
    			autonomousCase++;
    		}
    		break;
    	case 2:
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
        		SmartDashboard.putBoolean("Swing test complete", true);
        		autonomousCase++;
    		}
    		break;
    	case 3:
    		break;
    	}
    }
    public void switchShotTest()
    {
    	switch(autonomousCase)
    	{
        case 0: // Launch cube into switch with the short shot
    		m_ThePult.Launch();
    			
			autonomousWait = 0;
			autonomousCase++;
			break;
    	case 1:
    		if(autonomousWait >= SWITCH_CATAPULT_DELAY)
    		{
    			m_ThePult.Arm();
        		autonomousCase++;
    		}
    		break;
    	case 2:
    		break;
    	}
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
            		
            		//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower++;
            		autonomousWait = 0;
        		}
        		else
        		{
        			//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
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
            		
            		//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
            		arrIndex++;
            		
            		driveTrainDiagnosticPower--;
            		autonomousWait = 0;
        		}
        		else
        		{
        			//m_NetworkTable.putNumberArray("diagnosticLeftRate", diagnosticLeftRate);
        			//m_NetworkTable.putNumberArray("diagnosticRightRate", diagnosticRightRate);
        			//m_NetworkTable.putNumberArray("diagnosticPower", diagnosticPowerSent);
        			
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
    
    // TODO Straight drive length?
    public void switchCenter()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Swing to move toward the desired switch plate
    		if(switchChar == 'L')
    		{
        		m_DriveTrain.SetSwingParameters(-30, false);
    		}
    		else
    		{
    			m_DriveTrain.SetSwingParameters(30, true);
    		}
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1: // Drive toward desired switch plate and arm
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			if(switchChar == 'L')
    			{
            		m_DriveTrain.DriveControlledAngle(-12*4.5, 8, 5, -30);
    			}
    			else
    			{
    				m_DriveTrain.DriveControlledAngle(-12*4, 8, 5, 30);
    			}
    			
    			m_ThePult.Arm();
    			m_Elevator.setPosition(kHigh);
        		autonomousCase++;
    		}
    		break;
    	case 2: // Swing to square up with switch plate
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if(switchChar == 'L')
    			{
        			m_DriveTrain.SetSwingParameters(0, true);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3: // Launch cube into switch with the short shot
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			
    			m_ThePult.Launch();
    			
    			autonomousWait = 0;
    			autonomousCase++;
    		}
    		break;
    	case 4:
    		if(autonomousWait >= SWITCH_CATAPULT_DELAY)
    		{
    			m_ThePult.Arm();
    			autonomousCase++;
    		}
    		break;
    	case 5:
    		break;
    	}
    }
     
    //TODO Find actual distances/angles
    public void scaleLeftRight()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Path our routine
    		if(scaleChar == autonRoutine.toUpperCase().charAt(0))
				autonomousCase++; // ******Straight ahead
			else
	    		autonomousCase = 2;// Cross the field
    		break;
    	case 1: // ******Drive directly to the scale as we started on the same side as the scale
    			m_DriveTrain.DriveDistance(-(19*12/*Decision Point*/ + 70.5/*Decision point to scale*/), 4, 1);
    			m_Elevator.setPosition(kHigh);
    			autonomousCase = 8; // ******Jump to the end of the routine
    		break;
    	case 2: // Drive to the decision point
    		m_DriveTrain.DriveDistance(19*12, 4, 1);
    		m_Elevator.setPosition(kHigh);
    		autonomousCase++;
    		break;
    	case 3: // Turn to cross the field
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
        		if(scaleChar == 'R')
        		{
        			m_DriveTrain.SetSwingParameters(90, true);
        		}
        		else
        		{
        			m_DriveTrain.SetSwingParameters(-90, false);
        		}
        		m_DriveTrain.StartSwingTurn();
        		autonomousCase++;
    		}
    		break;
    	case 4: // Cross the field
    		if(m_DriveTrain.SwingAngleOnTarget())
    			m_DriveTrain.disableSwingPID();
    		if(scaleChar == 'R')
    		{
    			m_DriveTrain.DriveControlledAngle(-15*12, 5, 5, 90);
    		}
    		else
    		{
    			m_DriveTrain.DriveControlledAngle(-15*12, 5, 5, -90);
    		}
			autonomousCase++;
    		break;
    	case 5: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			}
    			autonomousCase++;
    		}
    		break;
    	case 6: // Finish turn PID and meet back up with the other decision path
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			autonomousCase++;
    		}
    		break;
    	case 7: // Drive up to scale from decision point or parallel decision point
    		m_DriveTrain.DriveDistance(-70.5, 4, 1);
    		autonomousCase++;
    		break;
    	case 8: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(-30, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(30, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			m_ThePult.Arm();
    			autonomousCase++;
    		}
    		break;
    	case 9: // Shoot powercube onto scale plate
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			
    			m_ThePult.Launch();
        		autonomousCase++;
    		}
    		break;
    	case 10:
    		break;
    	}
    }
    
    public void scaleCenter()
    {
    	// Scale R 	= 	-1
    	// Scale L 	= 	 1
    	//Switch R 	= 	 1
    	//Switch L 	= 	-1
    	switch(autonomousCase)
    	{
    	case 0: // Swing to move toward the desired scale
    		m_DriveTrain.ResetGyro();
    		if (scaleTurn == -1) 
    		{
    			m_DriveTrain.SetSwingParameters(45, true);
    		} 
    		else 
    		{
    			m_DriveTrain.SetSwingParameters(-45, false);
    		}
    		
    		m_DriveTrain.StartSwingTurn();
    		autonomousCase++;
    		break;
    	case 1: // Drive to prepare to run parallel with the wall
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			
        		m_DriveTrain.DriveControlledAngle(-8*12, 8, 5, -45*scaleTurn);
        		
        		m_Elevator.setPosition(kHigh);
        		
        		autonomousCase++;
    		}
    		break;
    	case 2: // Swing to be parallel with the wall
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if (scaleTurn == -1) 
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			} 
    			else 
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			}
    			
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3: // Drive to scale
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DriveControlledAngle(-6*12, 6, 8, 0);
    			
    			m_ThePult.Arm();
    			
    			autonomousCase++;
    		}
    		break;
    	case 4: // Swing to align for shooting
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			if (scaleTurn == -1) 
    			{
    				m_DriveTrain.SetSwingParameters(-30, false);
    			} 
    			else 
    			{
    				m_DriveTrain.SetSwingParameters(30, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 5: // Use the inferior siege engine
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			m_ThePult.Launch();
    			autonomousCase++;
    		}
    		break;
    	case 6: // Determine if we want to go for the switch
    		if(secondPart)
    		{
    			m_DriveTrain.disableSwingPID();
    			autonomousCase = 99;
    		} 
    		else 
    		{
    			
    			if (scaleTurn == -1) 
    			{
    				if (switchTurn == 1) 
    				{
    					m_DriveTrain.SetSwingParameters(45, false);
    				} 
    				else 
    				{
    					m_DriveTrain.SetSwingParameters(120, false);
    				}
    				
    			} 
    			else 
    			{
    				if (switchTurn == -1) 
    				{
    					m_DriveTrain.SetSwingParameters(-45, true);
    				} 
    				else 
    				{
    					m_DriveTrain.SetSwingParameters(-120, true);
    				}
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    			break;
    		}
    		break;
    	case 7:
    		if (m_DriveTrain.SwingAngleOnTarget()) 
    		{
    			m_ThePult.Arm();
    			
    			m_DriveTrain.DisablePIDControl();
    			
    			if (scaleTurn == -1) 
    			{
    				if (switchTurn == 1) 
    				{
    					m_DriveTrain.DriveControlledAngle(3 * 12, 5, 4, 45);
    				} 
    				else 
    				{
    					m_DriveTrain.DriveControlledAngle(8 * 12, 5, 10, 120);
    				}
    				
    			} 
    			else 
    			{
    				if (switchTurn == -1) 
    				{
    					m_DriveTrain.DriveControlledAngle(3 * 12, 5, 4, -45);
    				} 
    				else 
    				{
    					m_DriveTrain.DriveControlledAngle(8 * 12, 5, 10, -120);
    				}
    			}
    			
    			autonomousCase++;
    		}
    		break;
    		
    	case 8:
    		if (m_DriveTrain.isStraightPIDFinished()) 
    		{
    			m_DriveTrain.DisablePIDControl();
    			if (scaleTurn == -1) 
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			} 
    			else 
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			m_DriveTrain.StartSwingTurn();
    			
    			autonomousCase++;
    		}
    		break;
    	case 9:
    		if (m_DriveTrain.SwingAngleOnTarget()) 
    		{
    			m_DriveTrain.disableSwingPID();
    			m_DriveTrain.DisablePIDControl();
    			autonomousCase++;
    		}
    		break;
    	case 10:
    		m_DriveTrain.disableSwingPID();
			m_DriveTrain.DisablePIDControl();
    		m_DriveTrain.arcadeDrive(0, 0);
    		break;
    		
    	case 99:
    		m_DriveTrain.disableSwingPID();
    		m_DriveTrain.DisablePIDControl();
    		m_DriveTrain.arcadeDrive(0, 0);
    		break;
    	}
    	
    }
    
    public void worstCaseScenarioScaleLeftRightTest()
    {
    	switch(autonomousCase)
    	{
    	case 0: // Turn to cross the field
    		//if(m_DriveTrain.isStraightPIDFinished())
    		//{
        		if(scaleChar == 'R')
        		{
        			m_DriveTrain.SetSwingParameters(90, true);
        		}
        		else
        		{
        			m_DriveTrain.SetSwingParameters(-90, false);
        		}
        		m_DriveTrain.StartSwingTurn();
        		autonomousCase++;
    		//}
    		break;
    	case 1: // Cross the field
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
	    		if(scaleChar == 'R')
	    		{
	    			m_DriveTrain.DriveControlledAngle(-15*12, 5, 5, 90);
	    		}
	    		else
	    		{
	    			m_DriveTrain.DriveControlledAngle(-15*12, 5, 5, -90);
	    		}
				autonomousCase++;
    		}
    		break;
    	case 2: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(0, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(0, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			autonomousCase++;
    		}
    		break;
    	case 3: // Finish turn PID and meet back up with the other decision path
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			autonomousCase++;
    		}
    		break;
    	case 4: // Drive up to scale from decision point or parallel decision point
    		m_DriveTrain.DriveDistance(-70.5, 4, 1);
    		autonomousCase++;
    		break;
    	case 5: // Turn to face scale
    		if(m_DriveTrain.isStraightPIDFinished())
    		{
    			m_DriveTrain.DisablePIDControl();
    			
    			// TODO what is the actual angle
    			if(scaleChar == 'R')
    			{
    				m_DriveTrain.SetSwingParameters(-30, false);
    			}
    			else
    			{
    				m_DriveTrain.SetSwingParameters(30, true);
    			}
    			m_DriveTrain.StartSwingTurn();
    			
    			autonomousCase++;
    		}
    		break;
    	case 6: // Shoot powercube onto scale plate
    		if(m_DriveTrain.SwingAngleOnTarget())
    		{
    			m_DriveTrain.disableSwingPID();
    			
    			m_ThePult.Launch();
        		autonomousCase++;
    		}
    		break;
    	case 7:
    		break;
    
    	}
    }
    
    public void teleopPeriodic()
    {
    	/**
         * This function is called periodically during operator control
         */
    	m_Compressor.setClosedLoopControl(true);
    	
    	// For quick testing reset
    	if(autonomousCase != 0)
    	{
    		autonomousCase = 0;
    		m_DriveTrain.DisablePIDControl();
    	}
    	
    	// Subsystem methods
    	arcadeDrive();
    	elevatorControl();
    	intakeControl();
    	
    	// Other
    	
    	//Misc variable updates
    	GetDashboardData();
    	WriteDashboardData();
    	
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
    	// Unfortunately both drive motor groups must be inverted in order for the encoders to properly read (On Lil' Geek) which is why the inputs are inverted here
    	
    	
    	if(m_RobotInterface.GetDriverLeftTrigger() && !m_RobotInterface.GetDriverLeftBumper())
    	{
    	 	m_driveSpeed = 0.7;
    	}
    	else if (!m_RobotInterface.GetDriverLeftTrigger() && m_RobotInterface.GetDriverLeftBumper())
    	{
    	 	m_driveSpeed = 1.0;
    	}
    	// - | +
    	m_DriveTrain.ArcadeDrive(-m_RobotInterface.GetDriverLeftY()*m_driveSpeed, m_RobotInterface.GetDriverRightX()*m_driveSpeed);
    	
   }
    public void elevatorControl() {
    	
    	if (m_RobotInterface.GetOperatorButton(7) && !(m_Elevator.getPositionTarget() == kFloor)) 
    	{
    		m_Elevator.setPosition(kFloor);
    	} 
    	else if (m_RobotInterface.GetOperatorButton(8) && !(m_Elevator.getPositionTarget() == kTransfer)) 
    	{
    		m_Elevator.setPosition(kTransfer);
    	} 
    	else if (m_RobotInterface.GetOperatorButton(9) && !(m_Elevator.getPositionTarget() == kHigh)) 
    	{
    		m_Elevator.setPosition(kHigh);
    	} 
    	else if (m_RobotInterface.GetOperatorButton(10) && !(m_Elevator.getPositionTarget() == kLow)) 
    	{
    		m_Elevator.setPosition(kLow);
    	} 
    	else if (!m_Elevator.isPIDEnabled() && Math.abs(m_RobotInterface.GetOperatorJoystick().getRawAxis(1)) > .05) 
    	{
    		m_Elevator.disablePID();
    		// Elevator power is halved to prevent damage to the elevator when manually controlled
    		m_Elevator.manualControl(m_RobotInterface.GetOperatorJoystick().getRawAxis(1) * 0.5);
    	}
    }
    public void intakeControl() {
    	if (m_RobotInterface.GetOperatorButton(3) && !m_RobotInterface.GetOperatorButton(4)) 
    	{
    		m_Intake.IntakeCube();
    	} 
    	else if (m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3)) 
    	{
    		m_Intake.ReleaseCube();
    	} 
    	else if (m_RobotInterface.GetOperatorButton(6) && !m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3))
    	{
    		m_Intake.RotateRight();
    	} 
    	else if (m_RobotInterface.GetOperatorButton(5) && !m_RobotInterface.GetOperatorButton(4) && !m_RobotInterface.GetOperatorButton(3))
    	{
    		m_Intake.RotateLeft();
    	} 
    	else 
    	{
    		m_Intake.StopIntake();
    	}
    	
    	
    	if(m_RobotInterface.GetOperatorButton(1) && m_Intake.getSolenoidState())
    	{
    		m_Intake.retractIntake();
    	}
    	else if(!m_RobotInterface.GetOperatorButton(1) && !m_Intake.getSolenoidState())
    	{
    		m_Intake.expandIntake();
    	}
    }
    public void catapultControl() {
    	
    	//Right Trigger - Fire - Start 2 second delay before rearming
    	
    	if (m_RobotInterface.GetDriverRightTrigger() && m_catapultDelay == 0) 
    	{
    		m_ThePult.Launch();
    		m_catapultDelay = 100;
    	} 
    	else if (m_catapultDelay <= 0) 
    	{
    		m_catapultDelay = 0;
    		m_ThePult.Arm();
    	}
    	
    	if (m_catapultDelay > 0) {
    		m_catapultDelay--;
    	}
    	
    }
    
    public void switchCatapultShot()
    {
    	if(m_RobotInterface.GetDriverRightBumper() && isShotFinished)
    	{
    		isShotFinished = false;
    		m_ThePult.Launch();
    		m_shortCatapultDelay = 5;
    	}
    	else if(m_shortCatapultDelay <= 0)
    	{
    		isShotFinished = true;
    		m_shortCatapultDelay = 0;
    		m_ThePult.Arm();
    	}
    	
    	if (m_shortCatapultDelay > 0)
    		m_shortCatapultDelay--;
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
    	SmartDashboard.putNumber("Auton Case", autonomousCase);
    	SmartDashboard.putBoolean("isShotFinished", isShotFinished);
    	SmartDashboard.putNumber("Switch Catapult Delay", m_shortCatapultDelay);
    	m_DriveTrain.WriteDashboardData();
    	m_Elevator.WriteDashboardData();
    	m_ThePult.WriteDashboardData();
    	m_Intake.WriteDashboardData();
    }
}