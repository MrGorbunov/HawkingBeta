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
import frc.robot.util.Controls;
import java.util.Map;

public class RobotContainer {
  private Drivetrain drivetrain_;
  private SendableChooser<DriveType> driveTypeChooser_;

  private Controls controls_;

  private Button shifterButton_;

  private enum DriveType {
    TANK,
    ARCADE
  }

  public RobotContainer() {
    drivetrain_ = new Drivetrain();
    driveTypeChooser_ = new SendableChooser<>();
    driveTypeChooser_.setDefaultOption("Tank", DriveType.TANK);

    SmartDashboard.putData(driveTypeChooser_);

    shifterButton_ = new Button(controls_::getShifterButton);

    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    shifterButton_.whenPressed(() -> drivetrain_.toggleShifter());
  }

  private void configureDefaultCommands() {
    drivetrain_.setDefaultCommand(new SelectCommand(
        Map.ofEntries(
            Map.entry(DriveType.TANK, new DriveTank(controls_::getLeftDriverY,
                                                    controls_::getRightDriverY,
                                                    drivetrain_)),
            Map.entry(DriveType.ARCADE, new DriveArcade(controls_::getLeftDriverY,
                                                        controls_::getRightDriverX,
                                                        drivetrain_))
        ),
        driveTypeChooser_::getSelected
    ));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
