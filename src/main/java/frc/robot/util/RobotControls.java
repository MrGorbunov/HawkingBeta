package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;

/**
 * A loose wrapper for the joysticks to control the robot.
 * The all functions should not have any parameters so that
 * they can be used as a supplier for commands.
 */
public class RobotControls {

  private static final int LEFT_OPERATOR_JOYSTICK_ID = 3;
  private static final int RIGHT_OPERATOR_JOYSTICK_ID = 4;
  private static final int LEFT_DRIVER_JOYSTICK_ID = 0;
  private static final int RIGHT_DRIVER_JOYSTICK_ID = 1;

  private static final int kDriveShiftButton = 11; 

  
  private Joystick leftDriverJoystick 
      = new Joystick(LEFT_DRIVER_JOYSTICK_ID);
  private  Joystick rightDriverJoystick
      = new Joystick(RIGHT_OPERATOR_JOYSTICK_ID);

  private Joystick leftOperatorJoystick
      = new Joystick(LEFT_OPERATOR_JOYSTICK_ID);
  private Joystick rightOperatorJoystick
      = new Joystick(RIGHT_DRIVER_JOYSTICK_ID);

  public double getLeftDriverX() {
    return leftDriverJoystick.getX(Hand.kLeft);
  }

  public double getLeftDriverY() {
    return leftDriverJoystick.getY(Hand.kLeft);
  }

  public double getRightDriverX() {
    return rightDriverJoystick.getX(Hand.kRight);
  }

  public double getRightDriverY() {
    return rightDriverJoystick.getY(Hand.kRight);
  }

  public boolean getShifterButton() {
    return leftDriverJoystick.getRawButton(kDriveShiftButton);
  }
}
