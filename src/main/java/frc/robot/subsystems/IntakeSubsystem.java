package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorControllers;

public class IntakeSubsystem extends SubsystemBase
{
  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem()
  {

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public void intakeIn()
  {
    MotorControllers.IntakeMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void intakeOut()
  {
    MotorControllers.IntakeMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void intakeStop()
  {
    MotorControllers.IntakeMotor.stopMotor();
  }

  public void armUp()
  {
    if (MotorControllers.upperLimitSwitchForArm.get())
    {
      MotorControllers.ArmMotor.set(ControlMode.PercentOutput, 0.75);
      //MotorControllers.ArmMotor.stopMotor();
    } else {
      MotorControllers.ArmMotor.stopMotor();
      //MotorControllers.ArmMotor.set(ControlMode.PercentOutput, -0.6);
    }
  }

  public void armDown()
  {
    if (MotorControllers.lowerLimitSwitchForArm.get())
    {
      MotorControllers.ArmMotor.set(ControlMode.PercentOutput, -0.5);
    } else {
      MotorControllers.ArmMotor.stopMotor();
    }
  }

  public void armStop()
  {
    MotorControllers.ArmMotor.stopMotor();
  }

  //public boolean limitSwitchForClimbingArm()
  //{
  //  return Robot.limitSwitchForArm.get();
  //}
}
