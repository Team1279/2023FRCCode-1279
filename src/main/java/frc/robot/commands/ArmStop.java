// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class ArmStop extends CommandBase {

  private final Pneumatics stopPneumatic;
  private final WristSubsystem wristSubsystem;

  private final Timer m_timer = new Timer();

  /** Creates a new BackIntakeRetract. */
  public ArmStop(Pneumatics pneumatics, WristSubsystem wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stopPneumatic = pneumatics;
    this.wristSubsystem = wrist;
    addRequirements(this.stopPneumatic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stopPneumatic.ArmStop();
    wristSubsystem.wristStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}