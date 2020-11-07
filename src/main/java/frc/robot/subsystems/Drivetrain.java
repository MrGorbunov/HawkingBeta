package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX leftMaster_;
  private WPI_TalonSRX leftSlave_;
  private WPI_TalonSRX rightMaster_;
  private WPI_TalonSRX rightSlave_;

  private static final int kLeftMasterID = 1;
  private static final int kLeftSlaveID = 2;
  private static final int kRightMasterID = 3;
  private static final int kRightSlaveID = 4;

  public Drivetrain() {
    leftMaster_ = new WPI_TalonSRX(kLeftMasterID);
    leftSlave_ = new WPI_TalonSRX(kLeftSlaveID);
    rightMaster_ = new WPI_TalonSRX(kRightMasterID);
    rightSlave_ = new WPI_TalonSRX(kRightSlaveID);

    leftSlave_.follow(leftMaster_);
    rightSlave_.follow(rightMaster_);
  }

  @Override
  public void periodic() {

  }

  public void driveOpenLoop(double left, double right) {
    leftMaster_.set(left);
    rightMaster_.set(right);
  }
}
