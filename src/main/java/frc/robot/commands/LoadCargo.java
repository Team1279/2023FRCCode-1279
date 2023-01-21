package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.RobotContainer;

public class LoadCargo extends CommandBase
{
  private final IntakeSubsystem intakeSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;

  private final Timer m_timer = new Timer();
  /**
   * Creates a new KickerIn. This is the motor that allows for the Power Cells to go to the shooter
   */
  public LoadCargo(IntakeSubsystem intake, ConveyorSubsystem conveyor)
  {
    intakeSubsystem = intake;
    conveyorSubsystem = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

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
    intakeSubsystem.intakeIn();
    conveyorSubsystem.conveyorIn();
  }
/*
  // Called once the command ends or is interrupted.
  @Override
  public void cancel()
  {
    intakeSubsystem.intakeStop();
    conveyorSubsystem.conveyorStop();
  }
*/
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    intakeSubsystem.intakeStop();
    conveyorSubsystem.conveyorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
