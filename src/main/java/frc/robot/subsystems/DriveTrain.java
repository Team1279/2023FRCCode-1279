package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.Gamepads;
import frc.robot.MotorControllers;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class DriveTrain extends SubsystemBase
{
    public DifferentialDrive drive = new DifferentialDrive(MotorControllers.m_left, MotorControllers.m_right);
    //private static RobotLogger logger = RobotLogger.getInstance();
    private static DriveTrain s_instance = null;
    /**
     * In-place drivebase rotation controller
     */
    public PIDController m_turnController;
    /** Old angle for bump detection */
    private double oldYaw;
    /**
     * Left side gearbox.
     */
    //public TalonSRXCollection m_leftGearbox;

    /**
     * Right side gearbox.
     */
    //private TalonSRXCollection m_rightGearbox;

    /**
     * Left side encoder
     */
    //public EncoderBase m_leftEncoder;

    /**
     * Right side encoder
     */
    //private EncoderBase m_rightEncoder;

    /**
     * Odometry object for tracking robot position
     */
    //public DifferentialDriveOdometry m_odometry;

    /**
     * Pose2d for keeping track of robot position on the field
     */
    //public Pose2d m_robotPose = new Pose2d();

    /**
     * Velocity tracking vars
     */
    //private double m_lastLeftMeters, m_lastRightMeters, m_leftMPS, m_rightMPS = 0;

    /*
     * Drive Control Modes
     */
    //public enum DriveMode {
    //    OPEN_LOOP, // Open loop control (percent output control)
    //    VOLTAGE, // Voltage control

    //}

    // Keep track of the current DriveMode
    //private DriveMode m_currentDriveMode = DriveMode.OPEN_LOOP;

    //private DriveSignal m_currentSignal;
    
    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() 
    {

    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
    }

    private static int inverse = 1;
    private double modifier = 1;

    //static Joystick driverStick = Constants.DriverAndOperatorJoystick.driverStick;
    static Joystick driverStick = Gamepads.driverJoyStick;

    /**
     * This command drives the robot
     */
    public void robotDrive() 
    {
        double ySpeed = driverStick.getRawAxis(Constants.JoystickAxisIDs.LEFT_Y_AXIS) * -1 * inverse * modifier; // makes forward stick positive
        double zRotation = driverStick.getRawAxis(Constants.JoystickAxisIDs.RIGHT_X_AXIS) * modifier; 
            // WPI uses positive=> right; right stick for left and right

        drive.arcadeDrive(ySpeed, zRotation);

        drive.feed();
        //double flPos = MotorControllers.frontLeft.getSelectedSensorPosition();
        //double flVel = MotorControllers.frontLeft.getSelectedSensorVelocity();

        //System.out.println("Front Left Position: " + flPos);
        //System.out.println("Front Left Velocity: " + flVel);
    }

    public void driveForward() 
    {
        drive.arcadeDrive(0.4, 0);
        drive.feed();
    }

    public void driveBackward()
    {
        drive.arcadeDrive(-0.6, 0);
        drive.feed();
    }

    public void turnRobotRight() 
    {
        drive.arcadeDrive(0, 0.4);
        drive.feed();
    }

    public void turnRobotLeft() 
    {
        drive.arcadeDrive(0, -0.4);
        drive.feed();
    }

    public void turnRobotAround()
    {
        double flPos = MotorControllers.frontLeft.getSelectedSensorPosition();
        double flVel = MotorControllers.frontLeft.getSelectedSensorVelocity();
    }

    public void flipDirection() 
    {
        inverse = inverse * -1; // just flips the value between 1 and negative 1
    }

    /**
     * Sets the direction forward
     * Forward is the formalerly hatch side
     */
    public void setDirectionForward() 
    { // hatch side
        inverse = 1;
    }

    /**
     * Sets the direction backwards
     * Backwards is the formaleryly cargo side
     */
    public void setDirectionBack() 
    { // cargo side
        inverse = -1;
    }

    /**
     * This slows down the drive train
     * Slows it to 60%
     * Multiplies the drive train by 0.6
     */
    public void slowSpeed() 
    {
        modifier = 0.6; // 60%
    }

    /**
     * This is the normal speed
     * This will allow for the values to be times 1
     */
    public void normalSpeed() 
    {
        modifier = 1; // 100%
    }

    /**
     * Returns the direction of the drivetrain
     * 
     * @return false when not inverted, true when inverted
     */
    public static boolean getDirection() 
    {
        if (inverse == 1) {
            return false;
        }

        if (inverse == -1) {
            return true;
        }

        else
            return false;
    }

    public void stopDriving() 
    {
        drive.arcadeDrive(0, 0);
        drive.feed();
    }
    /**
     * public void complexDriveAuto(double distance)
     * {
     * 
     * }
     */

     /**
     * Get the DriveTrain instance.
     * 
     * @return DriveTrain instance
     */
    public static DriveTrain getInstance() {
        if (s_instance == null) {
            s_instance = new DriveTrain();
        }

        return s_instance;
    }

}
