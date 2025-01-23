package frc.maps;

public interface ControlMap {
  // Joystick Axises
  public static final int L_JOYSTICK_HORIZONTAL = 0;
  public static final int L_JOYSTICK_VERTICAL = 1;
  public static final int LT = 2;
  public static final int RT = 3;
  public static final int R_JOYSTICK_HORIZONTAL = 4;
  public static final int R_JOYSTICK_VERTICAL = 5;

  // Controller Buttons
  public static final int A_BUTTON = 1;
  public static final int B_BUTTON = 2;
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;
  public static final int LB_BUTTON = 5;
  public static final int RB_BUTTON = 6;
  public static final int SELECT_BUTTON = 7;
  public static final int START_BUTTON = 8;

  // These buttons are when you push down the left and right circle pad
  public static final int L_JOYSTICK_BUTTON = 9;
  public static final int R_JOYSTICK_BUTTON = 10;

  // D Pad Buttons
  public static final int DPAD_UP = 0;
  public static final int DPAD_UP_RIGHT = 45;
  public static final int DPAD_RIGHT = 90;
  public static final int DPAD_DOWN_RIGHT = 135;
  public static final int DPAD_DOWN = 180;
  public static final int DPAD_DOWN_LEFT = 225;
  public static final int DPAD_LEFT = 270;
  public static final int DPAD_UP_LEFT = 315;

  // Controller Zeroes
  public static final double ZERO = 0.15;
}
