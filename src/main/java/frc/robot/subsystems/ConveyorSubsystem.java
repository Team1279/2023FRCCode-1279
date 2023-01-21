package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorControllers;

public class ConveyorSubsystem extends SubsystemBase
{
  /**
   * Creates a new IntakeSubsystem.
   */
  public ConveyorSubsystem()
  {

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public void conveyorIn()
  {
    MotorControllers.ConveyorBeltMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void conveyorOut()
  {
    MotorControllers.ConveyorBeltMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void conveyorStop()
  {
    MotorControllers.ConveyorBeltMotor.stopMotor();
  }

  //public boolean limitSwitchForClimbingArm()
  //{
    //return Robot.limitSwitchForArm.get();
  //}
}
