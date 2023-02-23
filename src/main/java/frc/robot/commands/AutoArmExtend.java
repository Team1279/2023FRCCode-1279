// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoArmExtend extends CommandBase {

  private final Pneumatics extendPneumatic;
  private final WristSubsystem wristSubsystem;

  private final Timer m_timer = new Timer();

  /** Creates a new BackIntakeRetract. */
  public AutoArmExtend(Pneumatics pneumatics, WristSubsystem wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extendPneumatic = pneumatics;
    this.wristSubsystem = wrist;
    addRequirements(this.extendPneumatic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_timer.reset();
    m_timer.start();
    System.out.println("Start Extend Timer: " + m_timer.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      extendPneumatic.ArmExtend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}