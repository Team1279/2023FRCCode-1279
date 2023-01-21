package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorControllers;
import frc.robot.Constants;
import frc.robot.Gamepads;
import edu.wpi.first.wpilibj.Joystick;

public class ClimberSubsystem extends SubsystemBase
{
  /**
   * Creates a new IntakeSubsystem.
   */
  public ClimberSubsystem()
  {

  }

  static Joystick operatorStick = Gamepads.operatorJoyStick;

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public void climberExtend()
  {
    double leftTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.LEFT_TRIGGER_ID);
    double rightTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.RIGHT_TRIGGER_ID);

    if (leftTriggerVal > 0)
    {
      //System.out.println("Left Trigger Value: " + leftTriggerVal);
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput, leftTriggerVal);
    } else {
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
    if (rightTriggerVal > 0)
    {
      //System.out.println("Right Trigger Value: " + rightTriggerVal);
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput, rightTriggerVal);
    } else {
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void climberRetract()
  {
    double leftTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.LEFT_TRIGGER_ID);
    double rightTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.RIGHT_TRIGGER_ID);

    if (leftTriggerVal > 0)
    {
      //System.out.println("Left Trigger Value: " + leftTriggerVal);
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput, leftTriggerVal * -1.0);
    } else {
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
    if (rightTriggerVal > 0)
    {
      //System.out.println("Right Trigger Value: " + rightTriggerVal);
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput, rightTriggerVal * -1.0);
    } else {
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void climberStop()
  {
    MotorControllers.LeftClimberMotor.stopMotor();
    MotorControllers.RightClimberMotor.stopMotor();
  }

  public void rightClimberExtend()
  {
    double rightTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.RIGHT_TRIGGER_ID);

    if (rightTriggerVal > 0)
    {
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput, rightTriggerVal);
    } else {
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void rightClimberRetract()
  {
    double rightTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.RIGHT_TRIGGER_ID);

    if (rightTriggerVal > 0)
    {
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput, rightTriggerVal * -1.0);
    } else {
      MotorControllers.RightClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void rightClimberStop()
  {
    MotorControllers.RightClimberMotor.stopMotor();
  }

  public void leftClimberExtend()
  {
    double leftTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.LEFT_TRIGGER_ID);

    if (leftTriggerVal > 0)
    {
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput, leftTriggerVal);
    } else {
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void leftClimberRetract()
  {
    double leftTriggerVal = operatorStick.getRawAxis(Constants.JoystickAxisIDs.LEFT_TRIGGER_ID);

    if (leftTriggerVal > 0)
    {
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput, leftTriggerVal * -1.0);
    } else {
      MotorControllers.LeftClimberMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void leftClimberStop()
  {
    MotorControllers.LeftClimberMotor.stopMotor();
  }

}
