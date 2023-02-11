package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.MotorControllers;

public class SlowButtonPressed extends CommandBase
{
  private final DriveTrain driveTrain;

  //private final Timer m_timer = new Timer();
  /**
   * Creates a new KickerIn. This is the motor that allows for the Power Cells to go to the shooter
   */
  public SlowButtonPressed(DriveTrain DT)
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
    //double flPos = MotorControllers.frontLeft.getSelectedSensorPosition();
    //double flVel = MotorControllers.frontLeft.getSelectedSensorVelocity();
    //System.out.println("Front Left Position: " + flPos);
    //System.out.println("Front Left Velocity: " + flVel);
    //driveTrain.drive.arcadeDrive(1, 0);
    //driveTrain.drive.feed();
    //driveTrain.m_turnController.getD();
    //driveTrain.m_turnController.getI();
    //driveTrain.m_turnController.getP();
    //driveTrain.m_turnController.getPeriod();
    try
    {
        //public PIDController m_turnController;
        //public TalonSRXCollection m_leftGearbox;
        //public EncoderBase m_leftEncoder;
        //public DifferentialDriveOdometry m_odometry;
        //public Pose2d m_robotPose = new Pose2d();
        //driveTrain.m_leftEncoder.getSpeed()
        System.out.println("Left Speed = ");// + driveTrain.m_leftEncoder.getSpeed());
        //System.out.println("I = " + driveTrain.m_turnController.getI());
        //System.out.println("P = " + driveTrain.m_turnController.getP());
        //System.out.println("Periodic = " + driveTrain.m_turnController.getPeriod());
    } catch (Exception e)
    {
        System.out.println(e.getStackTrace());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    driveTrain.drive.arcadeDrive(0, 0);
    driveTrain.drive.feed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}

