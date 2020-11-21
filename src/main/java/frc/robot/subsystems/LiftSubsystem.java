package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Robot;
import frc.robot.util.SimEncoder;

public class LiftSubsystem extends SubsystemBase {

  private static final int kMasterMotorID = 8;
  private static final int kFollowerMotorID = 9;
  private static final int kNumMotors = 2;
  private static final DCMotor kMotors = DCMotor.getMiniCIM(kNumMotors);

  private static final double kCarriageMass = 4.5; // kg
  private static final double kPulleyRadius = 0.0381 / 2; // m
  private static final double kGearRatio = 8.0 / 1; // output / input

  private static final double kMaxSpeed = 1.5; // m / s
  private static final double kMaxAccel = 2.0; // m / s

  private static final double kModelPositionAccuracy = 0.0508; // m
  private static final double kModelSpeedAccuracy = 1.016; // m / s
  private static final double kEncoderAccuracy = 0.001;

  private static final double kPositionErrorTolerance = 0.0254; // m
  private static final double kVelocityErrorTolerance = 0.254; // m

  private static final double kControlEffortTolerance = 12.0; // Volts
  private static final double kMaxVoltage = 12.0; // Volts

  private static final double kMinHeight = 0.0; // meters
  private static final double kMaxHeight = 2.5; // meters

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      kMaxSpeed, kMaxAccel);

  private TrapezoidProfile.State lastReference = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0.0, 0.0);

  private final LinearSystem<N2, N1, N1> plant = LinearSystemId.createElevatorSystem(
      kMotors, kCarriageMass, kPulleyRadius,
      kGearRatio);

  private final KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(
      Nat.N2(), Nat.N1(),
      plant,
      VecBuilder.fill(kModelPositionAccuracy, kModelSpeedAccuracy),
      VecBuilder.fill(kEncoderAccuracy), Robot.LOOP_TIME);

  private final LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(
      plant,
      VecBuilder.fill(kPositionErrorTolerance, kVelocityErrorTolerance),
      VecBuilder.fill(kControlEffortTolerance),
      Robot.LOOP_TIME);

  private final LinearSystemLoop<N2, N1, N1> loop = new LinearSystemLoop<>(
      plant, controller, observer, 
      kMaxVoltage,
      Robot.LOOP_TIME);

  private final WPI_TalonSRX masterMotor = new WPI_TalonSRX(kMasterMotorID);
  private final WPI_TalonSRX followerMotor = new WPI_TalonSRX(kFollowerMotorID);

  private ElevatorSim sim = new ElevatorSim(plant,
      kMotors, kGearRatio, kPulleyRadius, kMinHeight, kMaxHeight);
  private SimEncoder encoderSim;

  /**
   * Constuct a Lift Subsystem.
   */
  public LiftSubsystem() {
    masterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    masterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        (int) (Robot.LOOP_TIME * 1000));
    followerMotor.follow(masterMotor);
    if (RobotBase.isSimulation()) {
      encoderSim = new SimEncoder("Lift Encoder");
      encoderSim.setDistance(0.0);
      encoderSim.setSpeed(0.0);
    }
    reset();
  }

  /**
   * Reset the contorel to use the current postion as the desired posotion.
   */
  public void reset() {
    double encoderPos = RobotBase.isSimulation() ? encoderSim.getDistance()
        : ticksToMeters(masterMotor.getSelectedSensorPosition());
    double encoderVel = RobotBase.isSimulation() ? encoderSim.getSpeed()
        : ticksToMeters(masterMotor.getSelectedSensorVelocity());
    
    loop.reset(VecBuilder.fill(encoderPos, encoderVel));
    lastReference = new TrapezoidProfile.State(encoderPos, encoderVel);
  }

  @Override
  public void periodic() {
    double encoderPos = RobotBase.isSimulation() ? encoderSim.getDistance()
        : ticksToMeters(masterMotor.getSelectedSensorPosition());

    lastReference = (new TrapezoidProfile(constraints, goal, lastReference))
        .calculate(Robot.LOOP_TIME);
    loop.setNextR(lastReference.position, lastReference.velocity);
    loop.correct(VecBuilder.fill(encoderPos));
    loop.predict(Robot.LOOP_TIME);
    masterMotor.setVoltage(loop.getU(0));
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput(masterMotor.get() * RobotController.getBatteryVoltage());
    sim.update(Robot.LOOP_TIME);

    encoderSim.setDistance(sim.getPositionMeters());

    // This causes an error from the getVelocityMeterPerSecond() function
    //encoderSim.setSpeed(sim.getVelocityMetersPerSecond());
  }

  /**
   * Set the goal posoiton for the lift.
   *
   * @param position The goal posotion in meters
   */
  public void setGoal(double position) {
    if (position <= kMaxHeight && position >= kMinHeight) {
      goal = new TrapezoidProfile.State(position, 0.0);
    }
  }

  private static double ticksToMeters(double ticks) {
    return ((2 * kPulleyRadius * Math.PI) / 4096) * ticks;
  }
}