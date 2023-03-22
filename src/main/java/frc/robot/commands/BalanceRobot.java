package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class BalanceRobot extends CommandBase
{
  private final DriveTrain driveTrain;
  private float initialPitch;
  private float currentPitch;

  //private final Timer m_timer = new Timer();
  /**
   * Creates a new KickerIn. This is the motor that allows for the Power Cells to go to the shooter
   */
  public BalanceRobot(DriveTrain DT)
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
    float skirtAngle = (float)34.25;
    float loweredSkirtAngle = (float)11.0;
    float raisedPlatformAngle = (float)15.0;
    float levelPlatformAngle = (float)2.5;
    initialPitch = RobotContainer.initialPitch;
    currentPitch = RobotContainer.currentPitch;
    float relativePitch = currentPitch - initialPitch;

    //driveTrain.driveForward(0.8);

    if (isTiltedUp(initialPitch, currentPitch))
    {
      // Robot is climbing up the Charging Station
      if (relativePitch >= raisedPlatformAngle)
      {
        driveTrain.driveForward(0.8);
      }
      if ((relativePitch >= loweredSkirtAngle) && (relativePitch < raisedPlatformAngle))
      { 
        driveTrain.driveForward(0.6);
      }
      if ((relativePitch > levelPlatformAngle) && (relativePitch < loweredSkirtAngle))
      { 
        driveTrain.driveForward(0.4);
      }
    }

      //Robot is level
      if ((relativePitch <= levelPlatformAngle) && (relativePitch >= (levelPlatformAngle * -1)))
      {
        driveTrain.stopDriving();
      }
    
    if (isTiltedDown(initialPitch, currentPitch))
    {
      // Robot is backing up the Charging Station
      if (relativePitch <= (raisedPlatformAngle * -1))
      {
        driveTrain.driveForward(0.8);
      }
      if ((relativePitch <= (loweredSkirtAngle * -1)) && (relativePitch > (raisedPlatformAngle * -1)))
      { 
        driveTrain.driveForward(0.6);
      }
      if ((relativePitch <= (levelPlatformAngle * -1)) && (relativePitch > (loweredSkirtAngle * -1)))
      { 
        driveTrain.driveForward(0.4);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    driveTrain.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

  public void balanceRobot(float initialPitch, float currentPitch)
  {
    float skirtAngle = (float)34.25;
    float loweredSkirtAngle = (float)11.0;
    float raisedPlatformAngle = (float)15.0;
    float levelPlatformAngle = (float)2.5;
    float relativePitch = currentPitch - initialPitch;

    if (isTiltedUp(initialPitch, currentPitch))
    {
      // Robot is climbing up the Charging Station
      if (relativePitch >= raisedPlatformAngle)
      {
        driveTrain.driveForward(0.8);
      }
      if ((relativePitch >= loweredSkirtAngle) && (relativePitch < raisedPlatformAngle))
      { 
        driveTrain.driveForward(0.6);
      }
      if ((relativePitch > levelPlatformAngle) && (relativePitch < loweredSkirtAngle))
      { 
        driveTrain.driveForward(0.4);
      }
    }

      //Robot is level
      if ((relativePitch <= levelPlatformAngle) && (relativePitch >= (levelPlatformAngle * -1)))
      {
        driveTrain.stopDriving();
      }
    
    if (isTiltedDown(initialPitch, currentPitch))
    {
      // Robot is backing up the Charging Station
      if (relativePitch <= (raisedPlatformAngle * -1))
      {
        driveTrain.driveForward(0.8);
      }
      if ((relativePitch <= (loweredSkirtAngle * -1)) && (relativePitch > (raisedPlatformAngle * -1)))
      { 
        driveTrain.driveForward(0.6);
      }
      if ((relativePitch <= (levelPlatformAngle * -1)) && (relativePitch > (loweredSkirtAngle * -1)))
      { 
        driveTrain.driveForward(0.4);
      }
    }
  }

  public Boolean isTiltedUp(float initialPitch, float currentPitch)
  {
    Boolean tiltDetected = false;
    if (currentPitch > (initialPitch + 2.5))
    {
      tiltDetected = true;
    } else {
      tiltDetected = false;
    }
    return tiltDetected;
  }

  public Boolean isTiltedDown(float initialPitch, float currentPitch)
  {
    Boolean tiltDetected = false;
    if (currentPitch < (initialPitch - 2.5))
    {
      tiltDetected = true;
    } else {
      tiltDetected = false;
    }
    return tiltDetected;
  }
}
