package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveTank;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

public class RobotContainer {
  private Drivetrain drivetrain_;
  private SendableChooser<DriveType> driveTypeChooser_;

  private Joystick leftDriverJoystick_;
  private Joystick rightDriverJoystick_;

  private Joystick leftOperatorJoystick_;
  private Joystick rightOperatorJoystick_;

  private Button shifterButton_;

  private static final int kLeftDriverJoystickID = 0;
  private static final int kRightDriverJoystickID = 1;

  private static final int kLeftOperatorJoystickID = 3;
  private static final int kRightOperatorJoystickID = 4;

  private enum DriveType {
    TANK,
    ARCADE
  }

  public RobotContainer() {
    drivetrain_ = new Drivetrain();
    driveTypeChooser_ = new SendableChooser<>();
    driveTypeChooser_.setDefaultOption("Tank", DriveType.TANK);

    SmartDashboard.putData(driveTypeChooser_);

    leftDriverJoystick_ = new Joystick(kLeftDriverJoystickID);
    rightDriverJoystick_ = new Joystick(kRightDriverJoystickID);

    leftOperatorJoystick_ = new Joystick(kLeftOperatorJoystickID);
    rightOperatorJoystick_ = new Joystick(kRightOperatorJoystickID);

    shifterButton_ = new Button(this::getShifterButton);

    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    shifterButton_.whenPressed(() -> drivetrain_.toggleShifter());
  }

  private void configureDefaultCommands() {
    drivetrain_.setDefaultCommand(new SelectCommand(
        Map.ofEntries(
            Map.entry(DriveType.TANK, new DriveTank(this::getLeftDriverY,
                                                    this::getRightDriverY,
                                                    drivetrain_)),
            Map.entry(DriveType.ARCADE, new DriveArcade(this::getLeftDriverY,
                                                        this::getRightDriverX,
                                                        drivetrain_))
        ),
        driveTypeChooser_::getSelected
    ));
  }

  // TODO: Clean up
  private double getLeftDriverX() {
    return leftDriverJoystick_.getX(Hand.kLeft);
  }

  private double getLeftDriverY() {
    return leftDriverJoystick_.getY(Hand.kLeft);
  }

  private double getRightDriverX() {
    return rightDriverJoystick_.getX(Hand.kRight);
  }

  private double getRightDriverY() {
    return rightDriverJoystick_.getY(Hand.kRight);
  }

  private boolean getShifterButton() {
    return leftDriverJoystick_.getRawButton(11);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
