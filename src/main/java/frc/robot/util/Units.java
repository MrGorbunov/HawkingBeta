package frc.robot.util;

public class Units {
  public static double encoderTicksToMeters(double encoderTicks) {
    return encoderTicks * (0.478778824 / 4096.0);
  }

  public static double metersToEncoderTicks(double meters) {
    return meters * (4096.0 / 0.478778824);
  }
}
