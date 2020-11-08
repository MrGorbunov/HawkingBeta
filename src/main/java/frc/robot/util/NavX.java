/*
MIT License

Copyright (c) 2017 Team 254

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// Modifications made by FRC 4910 see original file at
// https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/lib/util/drivers/NavX.java

package frc.robot.util;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class NavX {
  protected class Callback implements ITimestampedDataSubscriber {
    @Override
    public void timestampedDataReceived(long systemTimestamp, long sensorTimestamp,
                                        AHRSUpdateBase update, Object context) {
      synchronized (NavX.this) {
        // This handles the fact that the sensor is inverted from our coordinate conventions.
        if (lastSensorTimestampMs != invalidTimestamp && lastSensorTimestampMs < sensorTimestamp) {
          yawRateDegreesPerSecond = 1000.0 * (-yawDegrees - update.yaw)
              / (double) (sensorTimestamp - lastSensorTimestampMs);
        }
        lastSensorTimestampMs = sensorTimestamp;
        yawDegrees = -update.yaw;
      }
    }
  }

  protected AHRS gyro;

  protected Rotation2d angleAdjustment = new Rotation2d();
  protected double yawDegrees;
  protected double yawRateDegreesPerSecond;
  protected final long invalidTimestamp = -1;
  protected long lastSensorTimestampMs;

  /**
   * Constructs new navx using SPI ports protocol.

   * @param spiPortId id of navx
   */
  public NavX(SPI.Port spiPortId) {
    gyro = new AHRS(spiPortId, (byte) 200);
    resetState();
    gyro.registerCallback(new Callback(), null);
  }

  /**
   * Constructs new navx using Seral port protocol.

   * @param serialPortId the id of the navx
   */
  public NavX(SerialPort.Port serialPortId) {
    gyro = new AHRS(serialPortId);
    resetState();
    gyro.registerCallback(new Callback(), null);
  }

  /**
   * Reset the NavX.
   */
  public synchronized void reset() {
    gyro.reset();
    resetState();
  }

  /**
   * Set the yaw to 0 and reset the NavX state.
   */
  public synchronized void zeroYaw() {
    gyro.zeroYaw();
    resetState();
  }

  /**
   * Resets the NavX.
   */
  private void resetState() {
    lastSensorTimestampMs = invalidTimestamp;
    yawDegrees = 0.0;
    yawRateDegreesPerSecond = 0.0;
  }

  /**
   * Set the angle to transform all reading by. 
   * Generally used for when the navx is mounted at an angle.

   * @param adjustment The Rotation2d to use for the adjustment
   */
  public synchronized void setAngleAdjustment(Rotation2d adjustment) {
    angleAdjustment = adjustment;
  }

  /**
   * Get the raw sensor value for the yaw of the navx.

   * @return The raw yaw reading of the navx in degrees.
   */
  protected synchronized double getRawYawDegrees() {
    return yawDegrees;
  }

  /**
   * Get the raw sensor value for the yaw of the navx.

   * @return The unadjusted yaw of navx as a Rotation2d.
   */
  public synchronized Rotation2d getRawRotation() {
    return Rotation2d.fromDegrees(getRawYawDegrees());
  }

  /**
   * Get the current rotation around the yaw axis relieve.
   * to the angle adjustment

   * @return The true yaw of the navx as a rotation 2d.
   */
  public synchronized Rotation2d getYaw() {
    return angleAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
  }

  /**
   * Get the rate at which the navx is moving on the yaw axis.

   * @return How fast the yaw is changing in degrees per sec.
   */
  public double getYawRateDegreesPerSec() {
    return yawRateDegreesPerSecond;
  }

  /**
   * Get the rate at which the navx is moving on the yaw axis.

   * @return How fast the yaw is changing in radians per sec.
   */
  public double getYawRateRadiansPerSec() {
    return 180.0 / Math.PI * getYawRateDegreesPerSec();
  }

  /**
   * Get measured acceleration of the navx on the x axis.

   * @return The navx acceleration reading on the x axis.
   */
  public double getRawAccelX() {
    return gyro.getRawAccelX();
  }

  /**
   * Get the roll of the navx in degrees.

   * @return The navx roll in degrees.
   */
  public double getRollDegrees() {
    return gyro.getRoll();
  }

  /**
   * Get the pitch of the sensor.

   * @return The navx pitch in degrees.
   */
  public double getPitchDegrees() {
    return gyro.getPitch();
  }

  /**
   * Gets the timestamp that we last heard from the navx sensor.

   * @return  The last time we got a message from the sensor.
   */
  public long getLastSensorTimestamp() {
    return lastSensorTimestampMs;
  }

  /**
   * Checks to see if the navx is currently connected.

   * @return True if the navx is connected otherwise false.
   */
  public boolean isConnected() {
    return gyro.isConnected();
  }
}
