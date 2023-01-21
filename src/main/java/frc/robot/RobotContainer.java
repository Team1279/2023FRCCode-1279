/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer; //used for Autonomous Mode
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  //private final DriveTrain m_robotDrive = new DriveTrain();
  public static DriveTrain m_robotDrive = new DriveTrain();

  public static ClimberSubsystem climber = new ClimberSubsystem();
  public static ConveyorSubsystem conveyor = new ConveyorSubsystem();
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static ShooterSubsystem shooter = new ShooterSubsystem();
  public static MotorControllers motors = new MotorControllers();

  //private PickupCargo pickupCargo = new PickupCargo(intake);
  //private ConveyorIn inConveyor = new ConveyorIn(conveyor);
  private EjectCargo ejectCargo = new EjectCargo(intake, conveyor, shooter);
  private LoadCargo loadCargo = new LoadCargo(intake, conveyor);
  private LowerClimbers lowerClimbers = new LowerClimbers(climber);
  private RaiseClimbers raiseClimbers = new RaiseClimbers(climber);
  //private RaiseLeftClimber raiseLeftClimber = new RaiseLeftClimber(climber);
  //private RaiseRightClimber raiseRightClimber = new RaiseRightClimber(climber);
  private RaiseIntake raiseIntake = new RaiseIntake(intake);
  private LowerIntake lowerIntake = new LowerIntake(intake);
  private ShootCargo shootCargo = new ShootCargo(shooter, conveyor);
  private AutoShootCargo autoShootCargo = new AutoShootCargo(shooter, conveyor);
  private DriveSlow driveSlow = new DriveSlow(m_robotDrive);
  private ShootLow shootLow = new ShootLow(shooter);

  private YButtonPressed yButton = new YButtonPressed(m_robotDrive);

  static Joystick operatorStick = Gamepads.operatorJoyStick;

  //public double leftClimberSpeed;
  //public double rightClimberSpeed;

  public int autoSetting = 0;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings()
  {
    //operator buttons
    //Gamepads.operator_A_Button.toggleWhenPressed(command)
    Gamepads.operator_A_Button.whenHeld(lowerIntake);
    Gamepads.operator_B_Button.whenHeld(raiseIntake);
    Gamepads.operator_X_Button.whenHeld(ejectCargo);
    //Gamepads.operator_Y_Button.whenPressed(yButton);
    Gamepads.operator_leftShoulderButton.whenHeld(loadCargo);
    Gamepads.operator_rightShoulderButton.whenHeld(shootCargo);
    Gamepads.operator_backButton.whenHeld(lowerClimbers);
    Gamepads.operator_startButton.whenHeld(raiseClimbers);
    Gamepads.operator_leftStickButton.whenHeld(lowerIntake); 
    Gamepads.operator_rightStickButton.whenHeld(raiseIntake); 
    Gamepads.operator_leftStickButton.whenHeld(shootLow);

    //driver buttons
    Gamepads.driver_rightShoulderButton.whenHeld(driveSlow);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An ExampleCommand will run in autonomous
    //return autoDriveCommand;
    //switch (Robot.getAutoNumber())
    //{
    //  case 0:
    //  return simpleAuto;
    //  default:
    //  return simpleAuto;
    //}
    return null;
  }

  public void setAutonomousSetting(int setting)
  {
    autoSetting = setting;
  }

  public void configureDriveTrain()
  {
    //edu.wpi.first.wpilibj.drive.DifferentialDrive.setRightSideInverted
    m_robotDrive.drive.setSafetyEnabled(false);
    m_robotDrive.drive.setExpiration(2);
    m_robotDrive.drive.setSafetyEnabled(false);
    //m_robotDrive.setRightSideInverted(false);
  }

  //This method forces all motors on the robot to stop
  public void resetSystem()
  {
    m_robotDrive.drive.arcadeDrive(0, 0);
    m_robotDrive.drive.feed();
    conveyor.conveyorStop();
    intake.armStop();
    intake.intakeStop();
    shooter.shooterStop();
    climber.climberStop();
    climber.rightClimberStop();
    climber.leftClimberStop();
  }

  //Setup 15 second autonomous mode with 1/2 second intervals
  public void driveAuto2(Timer timer)
  { 
    //Shoot Cargo
    if (timer.get() >= 0.2 && timer.get() < 2.0)
    {
      autoShootCargo.execute();
    }
    
    //Stop Shooting Cargo
    if (timer.get() >= 2.0 && timer.get() < 2.1)
    {
      //autoShootCargo.cancel();
      autoShootCargo.end(true);
    } 
    
    //Drive backwards
    if (timer.get() >= 2.0 && timer.get() < 2.2)
    {
      m_robotDrive.driveBackward();
    } 
    
    //Lower Intake Arms
    if (timer.get() >= 2.2 && timer.get() < 4.6)
    {
      lowerIntake.execute();
    } 
    
    //Stop Intake Arms
    if (timer.get() >= 4.6 && timer.get() < 4.7)
    {
      //lowerIntake.cancel();
      lowerIntake.end(true);
    } 
    
    //Load Cargo
    if (timer.get() >= 4.0 && timer.get() < 9.0)
    {
      loadCargo.execute();
    }
    
    //Stop Loading Cargo
    if (timer.get() >= 9.0 && timer.get() < 9.1)
    {
      //loadCargo.cancel();
      loadCargo.end(true);
    }
    
    //Turn Robot Right (0.6 secs is 180 degrees)
    if (timer.get() >= 3.0 && timer.get() < 5.1)
    {
      if (autoSetting == 0)
      {
        //System.out.println("Setting Switch = 0");
        m_robotDrive.turnRobotRight();
      } else {
        //System.out.println("Setting Switch = 1");
        m_robotDrive.turnRobotLeft();
      }
    }

    //Stop Turning
    if (timer.get() >= 5.1 && timer.get() < 5.2)
    {
      m_robotDrive.stopDriving();
    } 
    
    //Drive Robot Forward
    if (timer.get() >= 5.4 && timer.get() < 7.0)
    {
      m_robotDrive.driveForward();
    } 
    
    //Stop Driving Forward
    if (timer.get() >= 7.0 && timer.get() < 7.1)
    {
      m_robotDrive.stopDriving();
    } 

    //Drive Robot Backward
    if (timer.get() >= 7.2 && timer.get() < 7.8)
    {
      m_robotDrive.driveBackward();
    } 
    
    //Stop Driving Forward
    if (timer.get() >= 7.8 && timer.get() < 7.9)
    {
      m_robotDrive.stopDriving();
    } 
    
    //Turn Robot Left
    if (timer.get() >= 8.0 && timer.get() < 10.3)
    {
      //m_robotDrive.turnRobotLeft();

      if (autoSetting == 0)
      {
        m_robotDrive.turnRobotLeft();
        //System.out.println("Setting Switch = 0");
      } else {
        m_robotDrive.turnRobotRight();
        //System.out.println("Setting Switch = 1");
      }
    } 
    
    //Raise Intake Arm
    if (timer.get() >= 8.0 && timer.get() < 10.4)
    {
      raiseIntake.execute();
    } 
    
    //Stop Turning Left
    if (timer.get() >= 10.3 && timer.get() < 10.4)
    {
      m_robotDrive.stopDriving();
    } 
    
    //Drive Forward (back to shooting position)
    if (timer.get() >= 10.4 && timer.get() < 13.1)
    {
      m_robotDrive.driveForward();
    } 
    
    //Stop Driving Forward
    if (timer.get() >= 13.1 && timer.get() < 13.2)
    {
      m_robotDrive.stopDriving();
    } 
    
    //Lower Intake Arm
    if (timer.get() >= 11.0 && timer.get() < 11.7)
    {
      lowerIntake.execute();
    }

    //Stop Lowering Intake Arm
    if (timer.get() >= 12.0 && timer.get() < 12.1)
    {
      lowerIntake.end(true);
    }

    //Shoot Cargo
    if (timer.get() >= 12.5 && timer.get() < 14.9)
    {
      autoShootCargo.execute();
    } 
    
    //Stop Shooting Cargo
    if (timer.get() >= 14.9 && timer.get() < 15.0)
    {
      //autoShootCargo.cancel();
      autoShootCargo.end(true);
    } 
    /*
    //Drive Robot Backwards (Out of Tarmak)
    if (timer.get() >= 14.6 && timer.get() < 15.6)
    {
      m_robotDrive.driveBackward();
    } 
    
    //Stop Driving Robot Backwards
    if (timer.get() >= 15.6 && timer.get() < 15.7)
    {
      m_robotDrive.stopDriving();
    }
*/
    //Stop All Motors
    if (timer.get() >= 14.9)
    {
      m_robotDrive.drive.arcadeDrive(0, 0);
      m_robotDrive.drive.feed();
      m_robotDrive.stopDriving();
      conveyor.conveyorStop();
      intake.armStop();
      intake.intakeStop();
      shooter.shooterStop();
      climber.climberStop();
    }
    
  }

  //Setup 15 second autonomous mode with 1/2 second intervals
  public void driveAuto1(Timer timer)
  {
    if (timer.get() < 0.2)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 0.2 && timer.get() < 0.4)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 0.4 && timer.get() < 0.6)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 0.6 && timer.get() < 0.8)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 0.8 && timer.get() < 1.0)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 1.0 && timer.get() < 1.2)
    {
      autoShootCargo.execute();  
    } else if (timer.get() >= 1.2 && timer.get() < 1.4)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 1.4 && timer.get() < 1.6)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 1.6 && timer.get() < 1.8)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 1.8 && timer.get() < 2.0)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 2.0 && timer.get() < 2.2)
    {
      //autoShootCargo.cancel();
      autoShootCargo.end(true);
      m_robotDrive.driveBackward();
    } else if (timer.get() >= 2.2 && timer.get() < 2.4)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 2.4 && timer.get() < 2.6)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 2.6 && timer.get() < 2.8)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 2.8 && timer.get() < 3.0)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 3.0 && timer.get() < 3.2)
    {
      //m_robotDrive.stopDriving();
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
    } else if (timer.get() >= 3.2 && timer.get() < 3.4)
    {
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
    } else if (timer.get() >= 3.4 && timer.get() < 3.6)
    {
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
    } else if (timer.get() >= 3.6 && timer.get() < 3.8)
    {
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
    } else if (timer.get() >= 3.8 && timer.get() < 4.0)
    {
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
    } else if (timer.get() >= 4.0 && timer.get() < 4.2)
    {
      //m_robotDrive.stopDriving();
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
      loadCargo.execute();
    } else if (timer.get() >= 4.2 && timer.get() < 4.3)
    {
      //m_robotDrive.stopDriving();
      m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
      loadCargo.execute();
    } else if (timer.get() >= 4.3 && timer.get() < 4.6)
    {
      m_robotDrive.stopDriving();
      //m_robotDrive.turnRobotLeft();
      lowerIntake.execute();
      loadCargo.execute();
    } else if (timer.get() >= 4.6 && timer.get() < 4.8)
    {
      //m_robotDrive.turnRobotLeft();
      loadCargo.execute();
    } else if (timer.get() >= 4.8 && timer.get() < 5.0)
    {
      m_robotDrive.driveForward();
      loadCargo.execute();
    } else if (timer.get() >= 5.0 && timer.get() < 5.2)
    {
      m_robotDrive.driveForward();
      loadCargo.execute();
    } else if (timer.get() >= 5.2 && timer.get() < 5.3)
    {
      m_robotDrive.driveForward();
      //m_robotDrive.stopDriving();
      loadCargo.execute();
    } else if (timer.get() >= 5.3 && timer.get() < 5.6)
    {
      //m_robotDrive.driveForward();
      m_robotDrive.stopDriving();
      loadCargo.execute();
    } else if (timer.get() >= 5.6 && timer.get() < 5.8)
    {
      //m_robotDrive.driveForward();
      loadCargo.execute();
    } else if (timer.get() >= 5.8 && timer.get() < 6.0)
    {
      //m_robotDrive.driveForward();
      loadCargo.execute();
    } else if (timer.get() >= 6.0 && timer.get() < 6.2)
    {
      //m_robotDrive.driveForward();
      loadCargo.execute();
    } else if (timer.get() >= 6.2 && timer.get() < 6.4)
    {
      //m_robotDrive.driveForward();
      loadCargo.execute();
    } else if (timer.get() >= 6.4 && timer.get() < 6.6)
    {
      m_robotDrive.turnRobotRight();
      loadCargo.execute();
    } else if (timer.get() >= 6.6 && timer.get() < 6.8)
    {
      m_robotDrive.turnRobotRight();
      raiseIntake.execute();
    } else if (timer.get() >= 6.8 && timer.get() < 7.0)
    {
      m_robotDrive.turnRobotRight();
      raiseIntake.execute();
    } else if (timer.get() >= 7.0 && timer.get() < 7.2)
    {
      m_robotDrive.turnRobotRight();
      raiseIntake.execute();
    } else if (timer.get() >= 7.2 && timer.get() < 7.4)
    {
      m_robotDrive.stopDriving();
      //m_robotDrive.turnRobotRight();
      raiseIntake.execute();
    } else if (timer.get() >= 7.4 && timer.get() < 7.6)
    {
      //m_robotDrive.stopDriving();
      //m_robotDrive.turnRobotRight();
      raiseIntake.execute();
    } else if (timer.get() >= 7.6 && timer.get() < 7.8)
    {
      //m_robotDrive.stopDriving();
      //m_robotDrive.turnRobotRight();
    } else if (timer.get() >= 7.8 && timer.get() < 8.0)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 8.0 && timer.get() < 8.2)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 8.2 && timer.get() < 8.4)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 8.4 && timer.get() < 8.6)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 8.6 && timer.get() < 8.8)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 8.8 && timer.get() < 9.0)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 9.0 && timer.get() < 9.2)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 9.2 && timer.get() < 9.4)
    {
      m_robotDrive.driveForward();
    } else if (timer.get() >= 9.4 && timer.get() < 9.6)
    {
      //m_robotDrive.driveForward();
    } else if (timer.get() >= 9.6 && timer.get() < 9.8)
    {
      //m_robotDrive.driveForward();
    } else if (timer.get() >= 9.8 && timer.get() < 10.0)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 10.0 && timer.get() < 10.2)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 10.2 && timer.get() < 10.4)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 10.4 && timer.get() < 10.6)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 10.6 && timer.get() < 10.8)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 10.8 && timer.get() < 11.0)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 11.0 && timer.get() < 11.2)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 11.2 && timer.get() < 11.4)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 11.4 && timer.get() < 11.6)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 11.6 && timer.get() < 11.8)
    {
      autoShootCargo.execute();
    } else if (timer.get() >= 11.8 && timer.get() < 12.0)
    {
      //autoShootCargo.cancel();
      autoShootCargo.end(true);
      m_robotDrive.driveBackward();
    } else if (timer.get() >= 12.0 && timer.get() < 12.2)
    {
      m_robotDrive.driveBackward();
    } else if (timer.get() >= 12.2 && timer.get() < 12.4)
    {
      m_robotDrive.driveBackward();
    } else if (timer.get() >= 12.4 && timer.get() < 12.6)
    {
      m_robotDrive.driveBackward();
    } else if (timer.get() >= 12.6 && timer.get() < 12.8)
    {
      m_robotDrive.driveBackward();
    } else if (timer.get() >= 12.8 && timer.get() < 13.0)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 13.0 && timer.get() < 13.2)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 13.2 && timer.get() < 13.4)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 13.4 && timer.get() < 13.6)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 13.6 && timer.get() < 13.8)
    {
      //m_robotDrive.driveBackward();
    } else if (timer.get() >= 13.8 && timer.get() < 14.0)
    {
    } else if (timer.get() >= 14.0 && timer.get() < 14.2)
    {
    } else if (timer.get() >= 14.2 && timer.get() < 14.4)
    {
    } else if (timer.get() >= 14.4 && timer.get() < 14.6)
    {
    } else if (timer.get() >= 14.6 && timer.get() < 14.8)
    {
    } else if (timer.get() >= 14.8 && timer.get() < 15)
    {
    } else {
      m_robotDrive.drive.arcadeDrive(0, 0);
      m_robotDrive.drive.feed();
      m_robotDrive.stopDriving();
      conveyor.conveyorStop();
      intake.armStop();
      intake.intakeStop();
      shooter.shooterStop();
      climber.climberStop();
    }
  }
}