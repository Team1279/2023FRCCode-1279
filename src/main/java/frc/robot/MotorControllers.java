package frc.robot;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //Talon Motor Controllers
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; //Falcon 500 Controller
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;

/**
* Contains the Motor Controller definitions
*/
public class MotorControllers 
{
    /*******************************************/
    /* START OF MOTOR CONTOLLER CONFIGURATIONS */
    /*******************************************/
        /* Way to define a Motor Controller */
        /* public static WPI_TalonSRX nameOfTalon = new WPI_TalonSRX(TalonNumber); */
    
    /***********************************************/
    /* For competition drive train w/ Falcon 500's */
    /***********************************************/
    
    public static WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.ControllerIDs.FALCON_FRONT_LEFT_DRIVE_ID);
    public static WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.ControllerIDs.FALCON_REAR_LEFT_DRIVE_ID);
    public static WPI_TalonFX frontRight = new WPI_TalonFX(Constants.ControllerIDs.FALCON_FRONT_RIGHT_DRIVE_ID);
    public static WPI_TalonFX rearRight = new WPI_TalonFX(Constants.ControllerIDs.FALCON_REAR_RIGHT_DRIVE_ID);
    
    /***********************************************/
    /* For practice drive train w/ Sims and Talons */
    /***********************************************/
    /*
    public static WPI_TalonSRX frontLeft = new WPI_TalonSRX(Constants.ControllerIDs.TALON_FRONT_LEFT_DRIVE_ID);
    public static WPI_TalonSRX rearLeft = new WPI_TalonSRX(Constants.ControllerIDs.TALON_REAR_LEFT_DRIVE_ID);
    public static WPI_TalonSRX frontRight = new WPI_TalonSRX(Constants.ControllerIDs.TALON_FRONT_RIGHT_DRIVE_ID);
    public static WPI_TalonSRX rearRight = new WPI_TalonSRX(Constants.ControllerIDs.TALON_REAR_RIGHT_DRIVE_ID);
    */
    //public static WPI_VictorSPX rearLeft = new WPI_VictorSPX(Constants.ControllerIDs.TALON_REAR_LEFT_DRIVE_ID);
    /******************************/
    /* Game component controllers */
    /******************************/
    //Shooter
    public static WPI_TalonFX ShooterMotor = new WPI_TalonFX(Constants.ControllerIDs.SHOOTER_DRIVE_ID);
    //Conveyor Belt
    public static WPI_TalonSRX ConveyorBeltMotor = new WPI_TalonSRX(Constants.ControllerIDs.CONVEYOR_DRIVE_ID);
    //Arms Up & Down
    public static WPI_TalonSRX ArmMotor = new WPI_TalonSRX(Constants.ControllerIDs.ARM_DRIVE_ID);
    //Intake Wheel Bar
    public static WPI_TalonSRX IntakeMotor = new WPI_TalonSRX(Constants.ControllerIDs.INTAKE_DRIVE_ID);
    //public static WPI_VictorSPX IntakeMotor = new WPI_VictorSPX(Constants.ControllerIDs.INTAKE_DRIVE_ID);
    //Right Climber Motor
    public static WPI_TalonSRX RightClimberMotor = new WPI_TalonSRX(Constants.ControllerIDs.CLIMBER_RIGHT_DRIVE_ID); 
    //Left Climber Motor
    public static WPI_TalonSRX LeftClimberMotor = new WPI_TalonSRX(Constants.ControllerIDs.CLIMBER_LEFT_DRIVE_ID);
    //Kicker Motor
    public static WPI_TalonSRX KickerMotor = new WPI_TalonSRX(Constants.ControllerIDs.KICKER_MOTOR_DRIVE_ID);

    //Code for Vector Motor Controllers
    //public static WPI_VictorSPX frontRight = new WPI_VictorSPX(11);
    //public static WPI_VictorSPX rearRight = new WPI_VictorSPX(10);

    /*****************************************/
    /* END OF MOTOR CONTOLLER CONFIGURATIONS */
    /*****************************************/

    /********************************************************************************
     * These are the speed groups for the left & right sides of the robot for motors
     ********************************************************************************/
    public static MotorControllerGroup m_left = new MotorControllerGroup(frontLeft, rearLeft);
    public static MotorControllerGroup m_right = new MotorControllerGroup(frontRight, rearRight);

    /*******************************/
    /* LIMIT SWITCH CONFIGURATIONS */
    /*******************************/
    public static DigitalInput upperLimitSwitchForArm = new DigitalInput(Constants.Switches.armUpperLimitSwitch);
    public static DigitalInput lowerLimitSwitchForArm = new DigitalInput(Constants.Switches.armLowerLimitSwitch);

    /*************************/
    /* SWITCH CONFIGURATIONS */
    /*************************/
    public static DigitalInput autoSettingSwitch = new DigitalInput(Constants.Switches.autonomousSettingSwitch);
}
