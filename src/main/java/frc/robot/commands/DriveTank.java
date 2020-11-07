package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DriveTank extends CommandBase {
  private Supplier<Double> left_;
  private Supplier<Double> right_;
  private Drivetrain drivetrain_;

  public DriveTank(Supplier<Double> left, Supplier<Double> right, Drivetrain drivetrain) {
    left_ = left;
    right_ = right;

    drivetrain_ = drivetrain;
    addRequirements(drivetrain_);
  }

  @Override
  public void execute() {
    drivetrain_.driveOpenLoop(left_.get(), right_.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain_.driveOpenLoop(0.0, 0.0);
  }
}
