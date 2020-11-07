package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DriveArcade extends CommandBase {
  private Drivetrain drivetrain_;
  private Supplier<Double> throttle_;
  private Supplier<Double> steer_;

  public DriveArcade(Supplier<Double> throttle, Supplier<Double> steer, Drivetrain drivetrain) {
    throttle_ = throttle;
    steer_ = steer;

    drivetrain_ = drivetrain;
    addRequirements(drivetrain_);
  }

  @Override
  public void execute() {
    drivetrain_.driveOpenLoop(throttle_.get()+steer_.get(), throttle_.get()-steer_.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain_.driveOpenLoop(0.0, 0.0);
  }
}
