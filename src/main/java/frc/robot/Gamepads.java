package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
* Class that contains the driver and operator buttons
*/
public class Gamepads 
{
    //Create Drive Gamepad/Joystick
    public static Joystick driverJoyStick = new Joystick(Constants.GamePadIDs.DRIVER_GAMEPAD_ID);
    //Create Operator Gamepad/Joystick
    public static Joystick operatorJoyStick = new Joystick(Constants.GamePadIDs.OPERATOR_GAMEPAD_ID);

    public static Joystick getGamepad(int stickId)
    {
        if (stickId == Constants.GamePadIDs.DRIVER_GAMEPAD_ID)
        {
            return driverJoyStick;
        }
        if (stickId == Constants.GamePadIDs.OPERATOR_GAMEPAD_ID)
        {
            return operatorJoyStick;
        }
        return driverJoyStick; // failsafe
    }

    // Buttons are defined as follows:
    // static Button driver_ButtonName or operator_ButtonName = new JoystickButton(JoystickName, NumberOnController)
    //      static (allows for us to use in other classes)
    //      Button (Name of class)
    //      driver_ButtonName or operator_ButtonName (name of the button on the controller)
    //      new JoystickButton (name of the class for the Button)
    //      JoystickName (name of the joystick. will either be the driver or operator controller)
    //      NumberOnController (number that corresponds to the button on the controller)

    /********************
     * OPERATOR BUTTONS *
     ********************/
    static Button operator_A_Button = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.A_BUTTON_ID);
    static Button operator_B_Button = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.B_BUTTON_ID);
    static Button operator_X_Button = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.X_BUTTON_ID);
    static Button operator_Y_Button = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.Y_BUTTON_ID);
    static Button operator_leftShoulderButton = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.LEFT_SHOULDER_BUTTON_ID);
    static Button operator_rightShoulderButton = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.RIGHT_SHOULDER_BUTTON_ID);
    static Button operator_backButton = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.BACK_BUTTON_ID);
    static Button operator_startButton = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.START_BUTTON_ID);
    static Button operator_leftStickButton = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.LEFT_STICK_BUTTON_ID);
    static Button operator_rightStickButton = new JoystickButton(operatorJoyStick, Constants.ButtonIDs.RIGHT_STICK_BUTTON_ID);

    static POVButton operator_POVButton = new POVButton(operatorJoyStick, 0);
    static Trigger operator_leftTrigger = new Trigger();
    static Trigger operator_rightTrigger = new Trigger();

    /******************
     * DRIVER BUTTONS *
     ******************/
    static Button driver_A_Button = new JoystickButton(driverJoyStick, Constants.ButtonIDs.A_BUTTON_ID);
    static Button driver_B_Button = new JoystickButton(driverJoyStick, Constants.ButtonIDs.B_BUTTON_ID);
    static Button driver_X_Button = new JoystickButton(driverJoyStick, Constants.ButtonIDs.X_BUTTON_ID);
    static Button driver_Y_Button = new JoystickButton(driverJoyStick, Constants.ButtonIDs.Y_BUTTON_ID);
    static Button driver_leftShoulderButton = new JoystickButton(driverJoyStick, Constants.ButtonIDs.LEFT_SHOULDER_BUTTON_ID);
    static Button driver_rightShoulderButton = new JoystickButton(driverJoyStick, Constants.ButtonIDs.RIGHT_SHOULDER_BUTTON_ID);
    static Button driver_backButton = new JoystickButton(driverJoyStick, Constants.ButtonIDs.BACK_BUTTON_ID);
    static Button driver_startButton = new JoystickButton(driverJoyStick, Constants.ButtonIDs.START_BUTTON_ID);
    static Button driver_leftStickButton = new JoystickButton(driverJoyStick, Constants.ButtonIDs.LEFT_STICK_BUTTON_ID);
    static Button driver_rightStickButton = new JoystickButton(driverJoyStick, Constants.ButtonIDs.RIGHT_STICK_BUTTON_ID);

    static POVButton driver_POVButton = new POVButton(driverJoyStick, 0);
    static Trigger driver_leftTrigger = new Trigger();
    static Trigger driver_rightTrigger = new Trigger();
}
