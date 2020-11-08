package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private static final int LEFT_MASTER_ID = 1;
  private static final int LEFT_SLAVE_ID = 2;
  private static final int RIGHT_MASTER_ID = 3;
  private static final int RIGHT_SLAVE_ID = 4;
  private static final int SHIFTER_FORWARD_ID = 4;
  private static final int SHIFTER_BACKWARD_ID = 3;
  private static final DoubleSolenoid.Value HIGH_GEAR_STATE
      = DoubleSolenoid.Value.kForward; 
  private static final DoubleSolenoid.Value LOW_GEAR_STATE
      = DoubleSolenoid.Value.kReverse; 

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(LEFT_MASTER_ID);
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(LEFT_SLAVE_ID);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(RIGHT_MASTER_ID);
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(RIGHT_SLAVE_ID);

  private DoubleSolenoid shifter = new DoubleSolenoid(SHIFTER_FORWARD_ID, SHIFTER_BACKWARD_ID);

  /**
   * Default Constructor.
   * Configures followers to follow the master talons
   * and sets the drivetrain to high gear.
   */
  public DriveSubsystem() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    setShifter(true);
  }

  /**
   * Set the shifters to high gear or low gear.

   * @param highGear True if to use high gear false for low gear.
   */
  public void setShifter(boolean highGear) {
    shifter.set(highGear ? HIGH_GEAR_STATE : LOW_GEAR_STATE);
  }

  /**
   * Gets the current state of the shifter from the PCM.

   * @return True if in high gear false if in low gear.
   */
  public boolean getShifter() {
    return shifter.get() == HIGH_GEAR_STATE;
  }

  @Override
  public void periodic() {

  }

  /**
   * Drive the drivetrain in open loop (No feedback).

   * @param left The power [-1.0, 1.0] to supply to the right side.
   * @param right The power [-1.0, 1.0] to supply to the right side.
   */
  public void driveOpenLoop(double left, double right) {
    leftMaster.set(left);
    rightMaster.set(right);
  }
}
