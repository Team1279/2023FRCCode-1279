package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Gamepads;
import edu.wpi.first.wpilibj.Joystick;

public class RaiseClimbers extends CommandBase
{
  private final ClimberSubsystem climberSubSystem;

  private final Timer m_timer = new Timer();
  /**
   * Creates a new KickerIn. This is the motor that allows for the Power Cells to go to the shooter
   */
  public RaiseClimbers(ClimberSubsystem climber)
  {
    climberSubSystem = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  static Joystick operatorStick = Gamepads.operatorJoyStick;

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    climberSubSystem.climberExtend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    climberSubSystem.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
