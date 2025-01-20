package frc.helpers;

//import javax.naming.directory.DirContext;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.maps.ControlMap;

//This controllers are user input
//It also has the normalize method cuz IDK where else to put it
//To add another controller:
//  1. Copy the line that includes "Joystick contOne = new Joystick(0)"
//  2. Change "contOne" to "contTwo"
//  3. Change "Joystick(0)" to "Joystick(1)"
public class OI {
    public static Joystick[] joystickArray = {new Joystick(0), new Joystick(1)};

   

    //Returns whether or not the button is being pressed
    //The method takes in the RobotMap button
    //i.e "RobotMap.A_BUTTON"
    /**
     * 
     * @param index The index of the controller whose inputs should be used (Generally, 0 is for drive train and 1 is for mechanisms)
     * @param button Which button is being checked (Use ControlMap constants)
     * @return if the button is being pressed
     */
    public static boolean button(int index, int button){
        return joystickArray[index].getRawButton(button);
    }

    //DPad
    //returns an integer representation of direction
    //starts with up at 0 and increases clockwise
    //right is 90
    //upper left is 315
    //-1 if nothing
    /**
     * 
     * @param index The index of the controller whose inputs should be used (Generally, 0 is for drive train and 1 is for mechanisms)
     * @param direction the direction of the DPad that is being checked (Use ControlMap constants)
     * @return if the direction is being pressed on the DPad
     */
    public static boolean dPad(int index, int direction){
        return joystickArray[index].getPOV() == direction;
    }

    /**
     * 
     * @param index The index of the controller whose inputs should be used (Generally, 0 is for drive train and 1 is for mechanisms)
     * @return The direction in degrees of the DPad. 0 is up and increases in 45 degree increments clockwise. Returns -1 if DPad is not being pressed.
     */
    public static int dPadAng(int index){
        return joystickArray[index].getPOV();
    }

    //Returns how much the axises is being pushed or pulled down
    //The method takes in RobotMap axis
    //i.e. "RobotMap.LT"
    /**
     * 
     * @param index The index of the controller whose inputs should be used (Generally, 0 is for drive train and 1 is for mechanisms)
     * @param axis The axis that you are checking. On the X-Box controllers, LT and RT count as triggers. Use ControlMap constants.
     * @return how much the axises is being pushed or pulled down
     */
    public static double axis(int index, int axis){
        double axisVal = joystickArray[index].getRawAxis(axis);
        if(axisVal < ControlMap.ZERO && axisVal > -ControlMap.ZERO)
            return 0;
        else 
            return axisVal;
    }

    //Takes in a value and some bounds and forces it within those bounds
    //Used pretty much everywhere, so make sure you don't change anything to drastic.
    public static double normalize(double d, double min, double max){
        if(d > max)
            return max;
        else if(d < min)
            return min;
        else 
            return d;

       //return value > max ? max : value < min ? min : value;
    }

    public static void setRumble(int contNumber, double value){
        joystickArray[contNumber].setRumble(RumbleType.kLeftRumble, value);
        joystickArray[contNumber].setRumble(RumbleType.kRightRumble, value);
    }

    public static void setRumbleLeft(int contNumber, double value){
        joystickArray[contNumber].setRumble(RumbleType.kLeftRumble, value);
    }

    public static void setRumbleRight(int contNumber, double value){
        joystickArray[contNumber].setRumble(RumbleType.kRightRumble, value);
    }

}