package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class AutoShootCargo extends CommandBase
{
  private final ShooterSubsystem shooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;

  private final Timer m_timer = new Timer();
  /**
   * Creates a new ShootCargo. 
   */
  public AutoShootCargo(ShooterSubsystem shooter, ConveyorSubsystem conveyor)
  {
    shooterSubsystem = shooter;
    conveyorSubsystem = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    addRequirements(RobotContainer.conveyor);
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
    shooterSubsystem.Shoot();
    shooterSubsystem.feedShooter();
    conveyorSubsystem.conveyorIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void cancel()
  {
    shooterSubsystem.shooterStop();
    conveyorSubsystem.conveyorStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shooterSubsystem.shooterStop();
    conveyorSubsystem.conveyorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
