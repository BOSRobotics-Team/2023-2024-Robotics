package frc.lib.gyro;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Gyro hardware abstraction interface
 *
 * <p>The Gyro is modeled not as a subsystem but rather a shared resource that may be used by
 * multiple subsystems. Since it doesn't have a periodic method like a subsystem, it is important
 * that its updateInputs method is called just once via another periodic method. (This requires some
 * coordination, and, in this library, it is invoked via the drivetrain subsystem's periodic
 * mehtod.)
 *
 * <p>There is not a simulated version of this interface. Instead, the drivetrain supports
 * determining the robot's rotation from the gryo when connected and via the swwerve module
 * positions when not connected.
 */
public interface GyroIO extends Gyro {
  public static final int DRIVEGYRO_CCW = 0; // set gyro yaw to ccw for angle direction
  public static final int DRIVEGYRO_CW = 1; // set gyro yaw to clockwise for angle direction

  public static class GyroIOInputs {
    public boolean connected = false;
    public double headingDeg = 0.0;
    public double yawDeg = 0.0;
    public double pitchDeg = 0.0;
    public double rollDeg = 0.0;
    public double headingRateDegPerSec = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  public default boolean isConnected() {
    return false;
  }

  public default void setGyroDirection(int direction) {}

  public default int getGyroDirection() {
    return DRIVEGYRO_CCW;
  }

  /**
   * Set the robot's heading offset.
   *
   * @param offsetDegrees The offset to set to, in degrees on [-180, 180].
   */
  public default void setHeadingOffset(final double offsetDegrees) {}

  /**
   * Get the robot's heading offset.
   *
   * @return The offset to set to, in degrees on [-180, 180].
   */
  public default double getHeadingOffset() {
    return 0.0;
  }

  /** Zero the robot's heading. */
  public default void reset() {}

  public default double getYaw() {
    return 0.0;
  }

  public default double getPitch() {
    return 0.0;
  }

  public default double getRoll() {
    return 0.0;
  }
}
