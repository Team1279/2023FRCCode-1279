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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cscore.UsbCamera;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
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

  //AHRS ahrs;
  AHRS ahrs = new AHRS(I2C.Port.kMXP);
  //Boolean isTilted;
  float initialPitch = Float.parseFloat("0.0");
  boolean motionDetected;

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

    // Setup NavX
    //isTilted = false;
    //ahrs = new AHRS(SerialPort.Port.kMXP); // Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB 
    //ahrs = new AHRS(SPI.Port.kMXP);
    ////ahrs = new AHRS(I2C.Port.kMXP);
    //ahrs.calibrate();
    //ahrs.reset();
    //initialPitch = ahrs.getPitch();
    //initialPitch = 0.0;
    m_robotContainer.initialPitch = initialPitch;
    motionDetected = false;
    updateSmartDashboard();
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
      System.out.println("Setting Switch = 1");
    } else {
      m_robotContainer.setAutonomousSetting(0);
      System.out.println("Setting Switch = 0");
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
    //    m_robotContainer.driveAutoCS(timer);
    //    break;
    //  default:
    //    // Put default auto code here
    //    break;
    //}
    //System.out.println("Robot Timer: " + timer.get());

    //if (timer.get() < 0.2)
    //{
      //initialPitch = ahrs.getPitch();
      //System.out.println("Setting Initial Pitch = " + initialPitch);
      //m_robotContainer.initialPitch = initialPitch;
    //}
    updateSmartDashboard();
    //SmartDashboard.updateValues();
    motionDetected = ahrs.isMoving();
    //Balance the robot on the Charging Station
    m_robotContainer.initialPitch = initialPitch;
    m_robotContainer.currentPitch = ahrs.getPitch();

    //m_robotContainer.driveAutoCS(timer); //When lined up with Control Station
    m_robotContainer.driveAuto(timer); //Switch determines (1)Fwd&Balance or (0)Stop after Backwards
    //m_robotContainer.driveBalance(timer); //Just autobalance
    ////m_robotContainer.whichDriveAuto(timer); //Switch determines (1)Full Auto or (0)Skip Balancing
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

    //ahrs.calibrate();
    //ahrs.reset();
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
    //if (timer.get() < 0.2)
    //{
      //initialPitch = ahrs.getPitch();
      //m_robotContainer.initialPitch = initialPitch;
    //}
    updateSmartDashboard();
    //SmartDashboard.updateValues();
    motionDetected = ahrs.isMoving();
    //Balance the robot on the Charging Station
    m_robotContainer.initialPitch = initialPitch;
    m_robotContainer.currentPitch = ahrs.getPitch();
    //m_robotContainer.balanceRobot(initialPitch, ahrs.getPitch());
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

  public void updateSmartDashboard()
  {
    /* Display 6-axis Processed Angle Data                                      */
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    
    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */
    
    SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
    
    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
    
    SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
    SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
    SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
    SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */
    
    SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
    SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
    SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
    SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
    
    /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    /* NOTE:  These values are not normally necessary, but are made available   */
    /* for advanced users.  Before using this data, please consider whether     */
    /* the processed data (see above) will suit your needs.                     */
    
    SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
    SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
    SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
    SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
    SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
    SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
    SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
    SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
    SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
    SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
    
    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
    
    /* Sensor Board Information                                                 */
    SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
    
    /* Quaternion Data                                                          */
    /* Quaternions are fascinating, and are the most compact representation of  */
    /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    /* from the Quaternions.  If interested in motion processing, knowledge of  */
    /* Quaternions is highly recommended.                                       */
    SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
    SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
    SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
    SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
    
    /* Connectivity Debugging Support                                           */
    SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
    SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());

    SmartDashboard.putString(  "Robot Tilt",   isTilted());
    SmartDashboard.putNumber(   "Initial Pitch",        initialPitch);
    SmartDashboard.putBoolean(  "MotionDetected",       ahrs.isMoving());
    float relativePitch = ahrs.getPitch() - initialPitch;
    SmartDashboard.putNumber(   "Relative Pitch",       Math.abs(relativePitch));
    SmartDashboard.putNumber(   "Speed Value",          m_robotContainer.getSpeedValue());

  }

  public String isTilted()
  {
    String tiltDetected = "Level";
    if (ahrs.getPitch() > (initialPitch + 2.5))
    {
      tiltDetected = "Tilted Up";
    } else if (ahrs.getPitch() < (initialPitch - 2.5))
    {
      tiltDetected = "Tilted Down";
    } else {
      tiltDetected = "Level";
    }
    return tiltDetected;
  }

  
}
