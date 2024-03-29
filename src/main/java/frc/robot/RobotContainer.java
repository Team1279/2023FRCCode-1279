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
import frc.robot.subsystems.Pneumatics;

import frc.robot.commands.ArmExtend;
import frc.robot.commands.ArmRetract;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  public static float initialPitch;
  public static float currentPitch;
  public static double speedValue;

  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_robotDrive = new DriveTrain();

  public static WristSubsystem wrist = new WristSubsystem();

  public static MotorControllers motors = new MotorControllers();
   
  //Pneumatics Declare
  Pneumatics all_pneumatics = new Pneumatics();
  ArmExtend armExtend = new ArmExtend(all_pneumatics, wrist);
  ArmRetract armRetract = new ArmRetract(all_pneumatics, wrist);
  AutoArmExtend autoArmExtend = new AutoArmExtend(all_pneumatics, wrist);
  AutoArmRetract autoArmRetract = new AutoArmRetract(all_pneumatics, wrist);
  ArmStop armStop = new ArmStop(all_pneumatics, wrist);
  ConeGrab coneGrab = new ConeGrab(all_pneumatics);
  ConeRelease coneRelease = new ConeRelease(all_pneumatics);
  ConeStop coneStop = new ConeStop(all_pneumatics);
  CubeGrab cubeGrab = new CubeGrab(all_pneumatics);
  CubeRelease cubeRelease = new CubeRelease(all_pneumatics);
  CubeStop cubeStop = new CubeStop(all_pneumatics);
  BalanceRobot balanceRobot = new BalanceRobot(m_robotDrive);
  BalanceRobotPID balanceRobotPID = new BalanceRobotPID(m_robotDrive);
  
  private RaiseWrist raiseWrist = new RaiseWrist(wrist);
  private LowerWrist lowerWrist = new LowerWrist(wrist);

  private DriveSlow driveSlow = new DriveSlow(m_robotDrive);
  private DriveFast driveFast = new DriveFast(m_robotDrive);

  static Joystick operatorStick = Gamepads.operatorJoyStick;

  public int autoSetting = 0;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the button bindings
    configureButtonBindings();

    all_pneumatics.ArmStop();
    all_pneumatics.ConeStop();
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
    Gamepads.operator_X_Button.whenPressed(armRetract);
    Gamepads.operator_Y_Button.whenPressed(armExtend);

    //Gamepads.operator_Y_Button.whenReleased(armStop);
    //Gamepads.operator_X_Button.whenReleased(armStop);
    Gamepads.operator_A_Button.whileHeld(raiseWrist);
    Gamepads.operator_B_Button.whileHeld(lowerWrist);
    Gamepads.operator_leftShoulderButton.whileHeld(coneGrab);
    Gamepads.operator_rightShoulderButton.whileHeld(coneRelease);
    Gamepads.operator_leftShoulderButton.whenReleased(coneStop);
    Gamepads.operator_rightShoulderButton.whenReleased(coneStop);

    //driver buttons
    Gamepads.driver_rightShoulderButton.whenHeld(driveSlow);
    Gamepads.driver_leftShoulderButton.whenHeld(driveFast);
    Gamepads.driver_X_Button.whenHeld(balanceRobotPID);
    //Gamepads.driver_X_Button.whenReleased();
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

  public double getSpeedValue()
  {
    return speedValue;
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
 
    wrist.wristStop();
    all_pneumatics.ArmStop();
  }

  public void whichDriveAuto(Timer timer)
  {
    if (autoSetting == 1)
    {
      driveAuto(timer);
    } else if (autoSetting == 0)
    {
      driveBalance(timer);
    } else {
      driveAuto(timer);
    }

  }

  //Setup 15 second autonomous mode with variable intervals
  public void driveAuto(Timer timer) //No Charging Station
  { 
    //Raise Arms
    if (timer.get() >= 0.1 && timer.get() < 2.0)
    {
      autoArmExtend.execute();
    }
    
    //Drive forward (to align with Goal)
    //if (timer.get() >= 4.3 && timer.get() < 5.0)
    if (timer.get() >= 2.0 && timer.get() < 2.7)
    {
      m_robotDrive.driveForward();
    } 

    //Stop Driving Forward and Lower Wrist
    //if (timer.get() >= 5.0 && timer.get() < 5.2)
    if (timer.get() >= 2.7 && timer.get() < 3.6)
    {
      m_robotDrive.stopDriving();
      lowerWrist.execute();
    } 

    //Stop Wrist
    //if (timer.get() >= 5.2 && timer.get() < 5.3)
    if (timer.get() >= 3.6 && timer.get() < 3.7)
    {
      lowerWrist.cancel();
      wrist.wristStop();
    }
    
    //Release Cone
    //if (timer.get() >= 5.2 && timer.get() < 5.4)
    if (timer.get() >= 3.7 && timer.get() < 3.9)
    {
      coneRelease.execute();
    }
    
    //Stop Release Cone
    //if (timer.get() >= 5.4 && timer.get() < 5.5)
    if (timer.get() >= 3.9 && timer.get() < 4.0)
    {
      coneRelease.cancel();
      coneStop.execute();
    }
    
    //Drive Backwards
    //if (timer.get() >= 5.6 && timer.get() < 8.0)
    if (timer.get() >= 4.0 && timer.get() < 7.1)
    {
      m_robotDrive.driveBackward();
    }

    //Raise Wrist
    //if (timer.get() >= 5.6 && timer.get() < 6.4)
    if (timer.get() >= 4.5 && timer.get() < 6.1)
    {
      raiseWrist.execute();
      autoArmRetract.execute();
    }

    //Stop Wrist & Arms
    //if (timer.get() >= 6.5 && timer.get() < 7.0)
    if (timer.get() >= 6.1 && timer.get() < 6.2)
    {
      raiseWrist.cancel();
      wrist.wristStop();
      autoArmRetract.cancel();
    } 

    //Stop Driving Backward
    //if (timer.get() >= 8.0 && timer.get() < 8.2)
    if (timer.get() >= 7.1 && timer.get() < 7.2)
    {
      m_robotDrive.stopDriving();
    }
    
    /* For full Auto (no Self-Balance)
    //Drive Robot Forward
    //if (timer.get() >= 10.0 && timer.get() < 13.0)
    if (timer.get() >= 7.2 && timer.get() < 13.35)
    {
      m_robotDrive.driveForward();
    } 
    
    //Stop Driving Forward
    //if (timer.get() >= 14.0 && timer.get() < 14.1)
    if (timer.get() >= 13.35 && timer.get() < 13.9)
    {
      m_robotDrive.stopDriving();
    } 
    */

    //Move forward and autobalance if autononmous physical switch is set to OFF
    if (autoSetting == 1)
    {
      if (timer.get() >= 7.2 && timer.get() < 8.8)
      {
        m_robotDrive.driveForward(0.6);
      }

      if (timer.get() >= 8.9)
      {
        //System.out.println("Auto-Balance in AUTONOMOUS - InitialPitch = " + initialPitch);

        balanceRobotPID.execute();
      }
    }
    
    /*
    //Stop All Motors
    if (timer.get() >= 14.9)
    {
      m_robotDrive.drive.arcadeDrive(0, 0);
      m_robotDrive.drive.feed();
      m_robotDrive.stopDriving();
      wrist.wristStop();
      autoArmRetract.cancel();
      autoArmExtend.cancel();
    }
    */
    
  }

  public void driveBalance(Timer timer) //With Charging Station
  { 
    //System.out.println("Auto-Balance in AUTONOMOUS - InitialPitch = " + initialPitch);

    balanceRobotPID.execute();
    
  }

  
}