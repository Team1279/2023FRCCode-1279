package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorControllers;

public class WristSubsystem extends SubsystemBase
{
  /**
   * Creates a new IntakeSubsystem.
   */
  public WristSubsystem()
  {

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public void wristUp()
  {
    if (MotorControllers.upperLimitSwitchForWrist.get())
    {
      MotorControllers.WristMotor.set(ControlMode.PercentOutput, 0.75);
    } else {
      MotorControllers.WristMotor.stopMotor();
    }
  }

  public void wristDown()
  {
    if (MotorControllers.lowerLimitSwitchForWrist.get())
    {
      MotorControllers.WristMotor.set(ControlMode.PercentOutput, -0.5);
    } else {
      MotorControllers.WristMotor.stopMotor();
    }
  }

  public void wristStop()
  {
    MotorControllers.WristMotor.stopMotor();
  }
}
