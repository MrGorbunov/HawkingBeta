package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * A loose wrapper for the joysticks to control the robot.
 * The all functions should not have any parameters so that
 * they can be used as a supplier for commands.
 */
public class RobotControls {

  private static final int kLeftDriverJoystickID = 0;
  private static final int kRightDriverJoystickID = 1;
  private static final int kLeftOperatorJoystickID = 3;
  private static final int kRightOperatorJoystickID = 4;


  private static final int kDriveShiftButton = 3;
  private static final int kLiftDownButton = 14;
  private static final int kLiftSwitchButton = 15;
  private static final int kLiftLowScaleButton = 16;
  private static final int kLiftHighScaleButton = 11;

  private Joystick leftDriverJoystick;
  private  Joystick rightDriverJoystick;
  private Joystick leftOperatorJoystick;
  private Joystick rightOperatorJoystick;
  private XboxController xboxController;

  private boolean useXbox;

  /**
   * Controllers RobotControls.
   *
   * @param useXbox Weather to use an xBox controller or joysticks.
   */
  public RobotControls(boolean useXbox) {
    this.useXbox = useXbox;
    if (this.useXbox) {
      xboxController = new XboxController(kLeftDriverJoystickID);
    } else {
      leftDriverJoystick = new Joystick(kLeftDriverJoystickID);
      rightDriverJoystick = new Joystick(kRightDriverJoystickID);
      leftOperatorJoystick = new Joystick(kLeftOperatorJoystickID);
      rightOperatorJoystick = new Joystick(kRightOperatorJoystickID);
    }
  }
  
  public double getLeftDriverX() {
    return useXbox ? xboxController.getX(Hand.kRight) : leftDriverJoystick.getX(Hand.kLeft);
  }

  public double getLeftDriverY() {
    return useXbox ? -xboxController.getY(Hand.kLeft) : -leftDriverJoystick.getY(Hand.kLeft);
  }

  public double getRightDriverX() {
    return useXbox ? xboxController.getRawAxis(2) : rightDriverJoystick.getX(Hand.kRight);
  }

  public double getRightDriverY() {
    return useXbox ? -xboxController.getRawAxis(3) : rightDriverJoystick.getY(Hand.kRight);
  }

  public boolean getShifterButton() {
    return useXbox ? xboxController.getBumper(Hand.kRight) :
                     leftDriverJoystick.getRawButton(kDriveShiftButton);
  }

  public boolean getLiftDownButton() {
    return useXbox ? xboxController.getAButton() : leftDriverJoystick.getRawButton(kLiftDownButton);
  }

  public boolean getLiftSwitchButton() {
    return useXbox ? xboxController.getXButton() :
                     leftDriverJoystick.getRawButton(kLiftSwitchButton);
  }

  public boolean getLiftLowScaleButton() {
    return useXbox ? xboxController.getBButton() :
                     leftDriverJoystick.getRawButton(kLiftLowScaleButton);
  }

  public boolean getLiftHighScaleButton() {
    return useXbox ? xboxController.getYButton() :
                     leftDriverJoystick.getRawButton(kLiftHighScaleButton);
  }

}
