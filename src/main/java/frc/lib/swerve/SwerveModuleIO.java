package frc.lib.swerve;

/** Swerve module hardware abstraction interface. */
public interface SwerveModuleIO {
  public static class SwerveModuleID {
    public int moduleNumber; // the module number (0-3); primarily used for logging
    public String canBusID; // name of the CAN bus
    public int driveMotorID; // the CAN ID of the drive motor
    public int angleMotorID; // the CAN ID of the angle motor
    public int canCoderID; // the CAN ID of the CANcoder
    public double angleOffsetDeg; // the absolute offset of the angle encoder in degrees

    public SwerveModuleID(
        int moduleNumber,
        String canBusID,
        int driveMotorID,
        int angleMotorID,
        int canCoderID,
        double angleOffsetDeg) {
      this.moduleNumber = moduleNumber;
      this.canBusID = canBusID;
      this.driveMotorID = driveMotorID;
      this.angleMotorID = angleMotorID;
      this.canCoderID = canCoderID;
      this.angleOffsetDeg = angleOffsetDeg;
    }
  }

  public static class SwerveModuleIOInputs {
    double drivePositionDeg = 0.0;
    double driveDistanceMeters = 0.0;
    double driveVelocityMetersPerSec = 0.0;
    double driveAppliedPercentage = 0.0;
    double[] driveCurrentAmps = new double[] {};
    double[] driveTempCelsius = new double[] {};

    double angleAbsolutePositionDeg = 0.0;
    double anglePositionDeg = 0.0;
    double angleVelocityRevPerMin = 0.0;
    double angleAppliedPercentage = 0.0;
    double[] angleCurrentAmps = new double[] {};
    double[] angleTempCelsius = new double[] {};
  }
  /** return the module number */
  public default int getModuleNumber() {
    return 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified percentage of full power. */
  public default void setDriveMotorPercentage(double percentage) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity) {}

  /** Run the turn motor to the specified angle. */
  public default void setAnglePosition(double degrees) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setAngleBrakeMode(boolean enable) {}

  public default boolean isDriveMotorConnected() {
    return false;
  }

  public default boolean isAngleMotorConnected() {
    return false;
  }

  public default boolean isAngleEncoderConnected() {
    return false;
  }
}
