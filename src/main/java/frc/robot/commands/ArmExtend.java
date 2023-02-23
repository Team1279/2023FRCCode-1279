// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ArmExtend extends CommandBase {

  private final Pneumatics extendPneumatic;
  private final WristSubsystem wristSubsystem;

  private final Timer m_timer = new Timer();

  /** Creates a new BackIntakeRetract. */
  public ArmExtend(Pneumatics pneumatics, WristSubsystem wrist) {
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
  public void execute() {
    if (m_timer.get() >= 0.0 && m_timer.get() <= 1.0)
    {
      extendPneumatic.ArmExtend();
    } else if (m_timer.get() > 1.0 && m_timer.get() <= 1.6)
    {
      System.out.println("Extend Timer: " + m_timer.get());
      extendPneumatic.ArmExtend();
      wristSubsystem.wristDown();
    } else {
      wristSubsystem.wristStop();
      extendPneumatic.ArmStop();
    }
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