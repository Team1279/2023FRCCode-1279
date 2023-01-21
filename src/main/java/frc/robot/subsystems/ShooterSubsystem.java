package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorControllers;

public class ShooterSubsystem extends SubsystemBase
{

  private double modifier = 1;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem()
  {

  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public void Shoot()
  {
    double motorSpeed = 0.9 * modifier;
    MotorControllers.ShooterMotor.set(ControlMode.PercentOutput, motorSpeed);
    //MotorControllers.KickerMotor.set(ControlMode.PercentOutput, 1.0);
    //MotorControllers.ConveyorBeltMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void feedShooter()
  {
    MotorControllers.KickerMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void BackOut()
  {
    MotorControllers.ShooterMotor.set(ControlMode.PercentOutput, -1.0);
    MotorControllers.KickerMotor.set(ControlMode.PercentOutput, -1.0);
    //MotorControllers.ConveyorBeltMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void shooterStop()
  {
    MotorControllers.ShooterMotor.stopMotor();
    MotorControllers.KickerMotor.stopMotor();
    //MotorControllers.ConveyorBeltMotor.stopMotor();
  }

  public void slowSpeed() 
  {
        modifier = 0.6; // 60%
  }

  public void normalSpeed() 
  {
        modifier = 1.0; // 100%
  }

  //public boolean limitSwitchForClimbingArm()
  //{
    //return Robot.limitSwitchForArm.get();
  //}
}
