package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class DriveFast extends CommandBase
{
  private final DriveTrain driveTrain;

  //private final Timer m_timer = new Timer();
  /**
   * Creates a new KickerIn. This is the motor that allows for the Power Cells to go to the shooter
   */
  public DriveFast(DriveTrain DT)
  {
    driveTrain = DT;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    //m_timer.reset();
    //m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    driveTrain.fastSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    driveTrain.normalSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
