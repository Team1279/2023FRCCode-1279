/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final class ControllerIDs
    {
        //This is where you can define the ID's for each motor controller on the robot
        public static final int TALON_FRONT_RIGHT_DRIVE_ID = 15; 
        public static final int TALON_REAR_RIGHT_DRIVE_ID = 16; 
        public static final int TALON_FRONT_LEFT_DRIVE_ID = 17; 
        public static final int TALON_REAR_LEFT_DRIVE_ID = 18; 
        public static final int FALCON_FRONT_RIGHT_DRIVE_ID = 17; //Falcon 500
        public static final int FALCON_REAR_RIGHT_DRIVE_ID = 16; //Falcon 500
        public static final int FALCON_FRONT_LEFT_DRIVE_ID = 15; //Falcon 500
        public static final int FALCON_REAR_LEFT_DRIVE_ID = 14; //Falcon 500

        public static final int WRIST_DRIVE_ID = 9; 
        
        public static final int CONVEYOR_DRIVE_ID = 5;
        public static final int ARM_DRIVE_ID = 6; 
        public static final int INTAKE_DRIVE_ID = 7; 
        public static final int CLIMBER_RIGHT_DRIVE_ID = 8; 
        public static final int CLIMBER_LEFT_DRIVE_ID = 9; 
        public static final int KICKER_MOTOR_DRIVE_ID = 10; 
        //public static final int TALON_ELEVEN = 11; 
        public static final int SHOOTER_DRIVE_ID = 20; //Falcon 500
    }

    public static final class GamePadIDs
    {
        //This is where you can define the ID's for each motor controller on the robot
        public static final int DRIVER_GAMEPAD_ID = 0; 
        public static final int OPERATOR_GAMEPAD_ID = 1; 
    }

    public static final class PneumaticIDs
    {
        //This is where you can define the ID's for each pneumatic solenoid on the robot
        public static final int ARM_EXTEND_ID = 6; //6
        public static final int ARM_RETRACT_ID = 7; //7
        public static final int CONE_GRAB_ID = 4; 
        public static final int CONE_RELEASE_ID = 5;
        public static final int CUBE_GRAB_ID = 2; 
        public static final int CUBE_RELEASE_ID = 3;
    }
    
    public static final class ButtonIDs
    {
        public static final int A_BUTTON_ID = 1;
        public static final int B_BUTTON_ID = 2;
        public static final int X_BUTTON_ID = 3;
        public static final int Y_BUTTON_ID = 4;
        public static final int LEFT_SHOULDER_BUTTON_ID = 5;
        public static final int RIGHT_SHOULDER_BUTTON_ID = 6;
        public static final int BACK_BUTTON_ID = 7;
        public static final int START_BUTTON_ID = 8;
        public static final int LEFT_STICK_BUTTON_ID = 9;
        public static final int RIGHT_STICK_BUTTON_ID = 10;
    }

    public static final class JoystickAxisIDs
    {
        public static final int LEFT_X_AXIS = 0;
        public static final int LEFT_Y_AXIS = 1;
        public static final int RIGHT_X_AXIS = 4;
        public static final int RIGHT_Y_AXIS = 5;
        
        public static final int LEFT_TRIGGER_ID = 2;
        public static final int RIGHT_TRIGGER_ID = 3;
    }

    public static final class Switches
    {
        public static int wristUpperLimitSwitch = 8;
        public static int wristLowerLimitSwitch = 9;
        public static int armUpperLimitSwitch = 0;
        public static int armLowerLimitSwitch = 1;
        public static int autonomousSettingSwitch = 2;
    }

    /***************************************************************************
     *          END OF TEAM 1279 CONSTANTS                                     *
     ***************************************************************************/

    public static final class RobotLimits
    {
        // distance in inches the robot wants to stay from an object
        private static final double kHoldDistance = 12.0;

        // factor to convert sensor values to a distance in inches
        private static final double kValueToInches = 0.125;

        // proportional speed constant
        private static final double kP = 0.05;
        private static final int kUltrasonicPort = 0;
    }

    /**
     * Constants regarding the DriveTrain
     */
    public static class DriveTrain 
    {
        /**
        * Encoder constants
        */
        public static class Encoders 
        {

            /* Encoder slots */
            public static final int LEFT_ENCODER_SLOT = 1;
            public static final int RIGHT_ENCODER_SLOT = 1;

            /* Encoder phases */
            public static final boolean LEFT_SENSOR_PHASE = true;
            public static final boolean RIGHT_SENSOR_PHASE = false;

            /* Ticks per revolution of the encoder */
            public static final int PULSES_PER_REVOLUTION = 5760; // 4096;// 1024 // 2880;//1440; // (isCompBot())? 4096
                                                                    // : 1440;

        }

        /**
         * Component measurements
         */
        public static class Measurements 
        {
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0);
            public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

            public static final double DRIVEBASE_WIDTH = Units.inchesToMeters(28.0);
            public static final double DRIVEBASE_LENGTH = Units.inchesToMeters(28.0);

            public static final double GEAR_RATIO = 8.45;

            public static final int MOTOR_MAX_RPM = 5330; // For cim motors

        }

        public static int ALIGNMENT_EPSILON = 3;
    }

    /**
     * Constants regarding the intake
     */
    public static class Intake 
    {

        // Motor controller IDs
        public static final int INTAKE_ACTUATOR_TALON = 13;
        public static final int INTAKE_ROLLER_TALON = 14;

        public static final boolean INTAKE_ACTUATOR_TALON_INVERTED = false;
        public static final boolean INTAKE_ROLLER_TALON_INVERTED = true;

        // Sensors DIO ports
        public static final int INTAKE_LIMIT_BOTTOM = 0;
        public static final int INTAKE_LIMIT_TOP = 1;

        // PID values
        public static final double kPArm = 0.011111111111;
        public static final double kIArm = 0.0;
        public static final double kDArm = 0.0;

        public static final double ARM_TICKS_PER_DEGREE = 1000;

        public static final double ARM_UP_SPEED = -0.85;
        public static final double ARM_DOWN_SPEED = 0.35;

        public static final double ROLLER_SPEED = 0.4;
    }

    /**
     * Control Gains Measurements
     */
    public static class ControlGains 
    {

        // Feedforward Gains
        public static final double ksVolts = 1.02; // Practice Base 0.837; // MiniBot 2.37
        public static final double kvVoltsSecondsPerMeter = 7.01; // Practice Base 2.46; // 1.8 MiniBot 1.73
        public static final double kaVoltsSecondsSquaredPerMeter = 2.64; // Practice Base 0.0455; // 0.0231 MiniBot
                                                                         // .0304

        // Optimal Control Gain for driving
        public static final double kPDriveVel = 0.478;// 0.68; //0.478;
        public static final double kIDriveVel = 0.0;
        public static final double kDDriveVel = 0.008;

        // Optimal Control Gain for turning
        // 2.86 2.83 2.77 2.71 2.83 2.67 over shot
        public static final double kPTurnVel = 0.0088;//0.0028;// 0.008; /// 0.0085;// 0.030;
        public static final double kITurnVel = 0.01;//0.0;//0.01; // 0.07; // 0.12;
        public static final double kDTurnVel = 0.0106; // 0.0066

        // Basic P control for encoder-only distance driving
        public static final double kRP = 0.05;

        // P = 0.027 I = 0.1 D = 0.006

        // Closest: 3.34m

        // PID Controller
        public static PIDController turningPIDController = new PIDController(kPTurnVel, kITurnVel, kDTurnVel);

        public static PIDController drivePidController = new PIDController(kPTurnVel, kITurnVel, kDTurnVel);

        // DifferentialDriveKinematics allows for the use of the track length
        public static final double kTrackWidthMeters = 0.1524;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);

        // Max Trajectory of Velocity and Acceleration
        public static final double kMaxSpeedMetersPerSecond = 3; // This value will most likely need to be changed
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; // This value will most likely need to
                                                                                 // be
                                                                                 // changed

        // Ramsete Parameters (Not sure if this is nessacary for trajectory and may need
        // changes)
        public static final double kRamseteB = 2; // in meters
        public static final double kRamseteZeta = .7; // in Seconds

    }

    public static class Autonomous 
    {

        /**
         * Number of seconds to wait before robot is allowed to score
         */
        public static final double SCORE_LATE_DELAY = 5.0;

        // Vision-based distance P Gain
        public static final double VISION_DISTANCE_KP = -0.1;

        public static final double AUTO_TARGET_DISTANCE_EPSILON = 5.0;

    }
}