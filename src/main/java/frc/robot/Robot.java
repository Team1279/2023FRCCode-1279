// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer; //used for Autonomous Mode
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
//import edu.wpi.first.cameraserver.*;
//import frc.lib5k.utils.RobotLogger;
//import frc.lib5k.utils.RobotLogger.Level;
//import frc.lib5k.logging.USBLogger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private RobotContainer m_robotContainer;

  UsbCamera forwardCamera;
  UsbCamera backwardCamera;

  /* Robot I/O helpers */
	//RobotLogger logger = RobotLogger.getInstance();
	//USBLogger usbLogger;

  //private static final String kDefaultAuto = "driveAuto1";
  //private static final String kCustomAuto = "My Auto";
  //private String m_autoSelected;
  //private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // NetworkTableEntry xEntry;
  // NetworkTableEntry yEntry;

  // Place variables to be used and referenced by the robot code here
  // double counter = 0.0;

  Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    //CameraServer.startAutomaticCapture();
    forwardCamera = CameraServer.startAutomaticCapture(0); //.getInstance().startAutomaticCapture(0);
    backwardCamera = CameraServer.startAutomaticCapture(1);
    //CameraServer.startAutomaticCapture();

    forwardCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kAutoManage);
    forwardCamera.setFPS(60);
    forwardCamera.setResolution(640, 480);

    backwardCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kAutoManage);
    backwardCamera.setFPS(60);
    backwardCamera.setResolution(640, 480);

    // Enable USB logging
		//usbLogger = new USBLogger("RobotLogs-2022/live");
		//logger.enableUSBLogging(usbLogger);
    // Start the logger
		//logger.start(0.02);

    timer.stop();
    timer.reset();
    timer.start();
    m_robotContainer = new RobotContainer();
    m_robotContainer.resetSystem();

    //logger.log("Robot", "Constructing Commands", Level.kRobot);

    // This code adds a SmartDashboard menu of options
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    //m_autoSelected = SmartDashboard.getData("Auto choices").toString();

    // Example of adding subsystem to SmartDashboard
    // SmartDashboard.putData(m_aSubsystem);

    // Add Scheduler status to the SmartDashboard
    // SmartDashboard.putData(Scheduler.getInstance());

    // Get the default instance of NetworkTables that was created automatically
    // when your program starts
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    // NetworkTable table = inst.getTable("datatable");

    // Get the entries within that table that correspond to the X and Y values
    // for some operation in your program.
    // xEntry = table.getEntry("X");
    // yEntry = table.getEntry("Y");
  }

  double x = 0;
  double y = 0;

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and uncomment the getString line to get the auto name from the 
   * text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    //logger.log("Robot", "Match Number: " + DriverStation.getMatchNumber());
		//logger.log("Robot", "Autonomous started");

    m_robotContainer.configureDriveTrain();
    m_robotContainer.resetSystem();

    MotorControllers.frontLeft.configFactoryDefault();
    MotorControllers.frontRight.configFactoryDefault();
    MotorControllers.rearLeft.configFactoryDefault();
    MotorControllers.rearRight.configFactoryDefault();

    // adjust these so that when the stick is forward both of these are green
    MotorControllers.frontLeft.setInverted(false);
    MotorControllers.rearLeft.setInverted(false);
    MotorControllers.frontRight.setInverted(true); 
    MotorControllers.rearRight.setInverted(true);

    MotorControllers.frontLeft.setSafetyEnabled(false);
    MotorControllers.rearLeft.setSafetyEnabled(false);
    MotorControllers.frontRight.setSafetyEnabled(false);
    MotorControllers.rearRight.setSafetyEnabled(false);

    //Get autonomous switch setting
    if (MotorControllers.autoSettingSwitch.get())
    {
      m_robotContainer.setAutonomousSetting(1);
      //System.out.println("Setting Switch = 1");
    } else {
      m_robotContainer.setAutonomousSetting(0);
      //System.out.println("Setting Switch = 0");
    }

    //timer.stop();
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    //switch (m_autoSelected)
    //{
    //  case kCustomAuto:
    //    // Put custom auto code here
    //    System.out.println("In Custom Auto");
    //    break;
    //  case kDefaultAuto:
    //    m_robotContainer.driveAuto1(timer);
    //    break;
    //  default:
    //    // Put default auto code here
    //    break;
    //}
    //System.out.println("Robot Timer: " + timer.get());
    //m_robotContainer.driveAuto1(timer);
    m_robotContainer.driveAuto2(timer);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    //logger.log("Robot", "Match Number: " + DriverStation.getMatchNumber());
		//logger.log("Robot", "Teleop started");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }
    //System.out.println("In configureDriveTrain");

    
    timer.reset();
    timer.start();

    m_robotContainer.configureDriveTrain();
    m_robotContainer.resetSystem();

    MotorControllers.frontLeft.configFactoryDefault();
    MotorControllers.frontRight.configFactoryDefault();
    MotorControllers.rearLeft.configFactoryDefault();
    MotorControllers.rearRight.configFactoryDefault();

    // adjust these so that when the stick is forward both of these are green
    MotorControllers.frontLeft.setInverted(false);
    MotorControllers.rearLeft.setInverted(false);
    MotorControllers.frontRight.setInverted(true); 
    MotorControllers.rearRight.setInverted(true);

    MotorControllers.frontLeft.setSafetyEnabled(false);
    MotorControllers.rearLeft.setSafetyEnabled(false);
    MotorControllers.frontRight.setSafetyEnabled(false);
    MotorControllers.rearRight.setSafetyEnabled(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    // Increment counter every 20ms and show it on the SmartDashboard
    // SmartDashboard.putNumber("Counter", counter++);
    //SmartDashboard.putString("TestMessage", "In teleopPeriodic");

    m_robotContainer.m_robotDrive.robotDrive();

    // Using the entry objects, set the value to a double that is constantly
    // increasing. The keys are actually "/datatable/X" and "/datatable/Y".
    // If they don't already exist, the key/value pair is added.
    // xEntry.setDouble(x);
    // yEntry.setDouble(y);
    // x += 0.05;
    // y += 1.0;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() 
  {
    timer.reset();
    timer.stop();
    m_robotContainer.resetSystem();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() 
  {

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() 
  {
    // Run teleopInit() when in Test mode
    teleopInit();
    m_robotContainer.resetSystem();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() 
  {
    // Run teleopPeriodic() when in Test mode
    teleopPeriodic();
  }
}
