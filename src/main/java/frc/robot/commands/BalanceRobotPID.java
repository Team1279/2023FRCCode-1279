package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class BalanceRobotPID extends CommandBase
{
  private final DriveTrain driveTrain;
  private float initialPitch;
  private float currentPitch;

  //private final Timer m_timer = new Timer();
  /**
   * Creates a new KickerIn. This is the motor that allows for the Power Cells to go to the shooter
   */
  public BalanceRobotPID(DriveTrain DT)
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
    double lowSpeed = 0.25;
    double highSpeed = 0.6;

    //Calculate Pitch Ratio
    //if relativePitch = raisedPlatformAngle then pitch ratio = 1.0 & speed = highSpeed
    //if relativePitch = levelPlatformAngle then pitch ratio = 0.0 & speed = 0.0;
    //speedRange = 0.8 - 0.2 = 0.6;
    //angleRange = 15.0 - 2.5 = 12.5;
    float pitchRatio = (Math.abs(relativePitch) - levelPlatformAngle) / (raisedPlatformAngle - levelPlatformAngle);

    double speedRatio = highSpeed - lowSpeed;
    double speedValue = (pitchRatio * speedRatio) + lowSpeed;

    //Set forward speed based on pitch and speed ratios
    // 2.5 < relativePitch <= 15.0
    if ((Math.abs(relativePitch) > levelPlatformAngle) && (Math.abs(relativePitch) <= raisedPlatformAngle))
    {
      System.out.println("Pitch level - Start Driving: " + relativePitch + "=>" + speedValue);
      if (isTiltedUp(initialPitch, currentPitch))
      {
        driveTrain.driveForward(speedValue);
      }
      if (isTiltedDown(initialPitch, currentPitch))
      {
        System.out.println("Tilted DOWN");
        driveTrain.driveBackward(speedValue);
      } 
    }
    //Robot pitch is at or above maximum platform angle, but less than skirt angle, go high speed
    // 15.0 < relativePitch <= 34.25
    else if ((Math.abs(relativePitch) > raisedPlatformAngle) && (Math.abs(relativePitch) <= skirtAngle))
    {
      if (isTiltedUp(initialPitch, currentPitch))
      {
        driveTrain.driveForward(highSpeed);
      }
      if (isTiltedDown(initialPitch, currentPitch))
      {
        driveTrain.driveBackward(highSpeed);
      }
    }
    //Robot is level so stop driving
    // 2.5 >= relativePitch
    else if ((Math.abs(relativePitch) <= levelPlatformAngle)) // && (relativePitch >= (levelPlatformAngle * -1)))
    {
      System.out.println("Robot is level - Stop Driving: " +  relativePitch + "=>" + speedValue);
      driveTrain.stopDriving();
    }
    //Robot at too steep of a pitch, so stop driving
    // 34.25 < relativePitch
    else if (Math.abs(relativePitch) > skirtAngle)
    {
      System.out.println("Robot is TO HIGH - Stop Driving: " +  relativePitch + "=>" + speedValue);
      driveTrain.stopDriving();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    //driveTrain.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
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
