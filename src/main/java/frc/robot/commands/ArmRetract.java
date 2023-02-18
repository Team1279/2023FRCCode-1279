// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class ArmRetract extends CommandBase {

  private final Pneumatics retractPneumatic;
  private final WristSubsystem wristSubsystem;

  private final Timer m_timer = new Timer();

  /** Creates a new BackIntakeRetract. */
  public ArmRetract(Pneumatics pneumatics, WristSubsystem wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.retractPneumatic = pneumatics;
    this.wristSubsystem = wrist;
    addRequirements(this.retractPneumatic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //System.out.println("Shooter Timer: " + m_timer.get());
    m_timer.reset();
    m_timer.start();
    //System.out.println("Shooter Timer: " + m_timer.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    retractPneumatic.ArmRetract();
    //System.out.println("Shooter Timer: " + m_timer.get());
    if (m_timer.get() > 0.0 && m_timer.get() <= 2.0)
    {
      //System.out.println("Shooter Timer: " + m_timer.get());
      wristSubsystem.wristUp();
    } else {
      wristSubsystem.wristStop();
      retractPneumatic.ArmStop();
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