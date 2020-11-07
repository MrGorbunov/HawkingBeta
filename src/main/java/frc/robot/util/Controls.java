package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;

public class Controls {
  private static Joystick leftDriverJoystick_;
  private static Joystick rightDriverJoystick_;

  private static Joystick leftOperatorJoystick_;
  private static Joystick rightOperatorJoystick_;

  private static final int kLeftDriverJoystickID = 0;
  private static final int kRightDriverJoystickID = 1;

  private static final int kLeftOperatorJoystickID = 3;
  private static final int kRightOperatorJoystickID = 4;

  private static Controls instance_;

  private Controls() {
    leftDriverJoystick_ = new Joystick(kLeftDriverJoystickID);
    rightDriverJoystick_ = new Joystick(kRightDriverJoystickID);

    leftOperatorJoystick_ = new Joystick(kLeftOperatorJoystickID);
    rightOperatorJoystick_ = new Joystick(kRightOperatorJoystickID);
  }

  public static Controls getInstance() { return instance_ == null ? (instance_ = new Controls()) : instance_; }

  public double getLeftDriverX() {
    return leftDriverJoystick_.getX(Hand.kLeft);
  }

  public double getLeftDriverY() {
    return leftDriverJoystick_.getY(Hand.kLeft);
  }

  public double getRightDriverX() {
    return rightDriverJoystick_.getX(Hand.kRight);
  }

  public double getRightDriverY() {
    return rightDriverJoystick_.getY(Hand.kRight);
  }

  public boolean getShifterButton() {
    return leftDriverJoystick_.getRawButton(11);
  }
}
