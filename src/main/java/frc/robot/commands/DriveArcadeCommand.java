package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

/**
 * This command drives the robot using the arcade control
 * layout.
 */
public class DriveArcadeCommand extends CommandBase {
  private DriveSubsystem drive;
  private Supplier<Double> throttle;
  private Supplier<Double> steer;

  /**
   * Constructor for DriveArcade Command.
   *
   * @param throttle The supplier for the command to read the throttle value.
   * @param steer The supplier for the command to read the steering value.
   * @param drive The instance of drive to use.
   */
  public DriveArcadeCommand(Supplier<Double> throttle,
                            Supplier<Double> steer,
                            DriveSubsystem drive) {
    this.throttle = throttle;
    this.steer = steer;

    this.drive = drive;
    addRequirements(this.drive);
  }

  @Override
  public void execute() {
    drive.driveOpenLoop(throttle.get() + steer.get(),
                             throttle.get() - steer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveOpenLoop(0.0, 0.0);
  }
}
