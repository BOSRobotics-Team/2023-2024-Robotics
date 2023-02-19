package frc.lib.gyro;

import static frc.robot.Constants.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class GyroIOPigeon2 implements GyroIO {
  private final WPI_Pigeon2 gyro;
  private final double[] xyzDps = new double[3];
  private final double[] yprdeg = new double[3];

  private boolean ccwHeading = false;
  private double headingOffset = 0.0;

  public GyroIOPigeon2(int id, String canBus) {
    gyro = new WPI_Pigeon2(id, canBus);
    gyro.configFactoryDefault();
    ccwHeading = true;

    initLogging();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    gyro.getRawGyro(xyzDps);
    gyro.getYawPitchRoll(yprdeg);

    inputs.connected = this.isConnected();
    inputs.yawDeg = yprdeg[0]; // degrees
    inputs.pitchDeg = yprdeg[1]; // degrees
    inputs.rollDeg = yprdeg[2]; // degrees
    inputs.headingDeg = headingOffset + (ccwHeading ? yprdeg[0] : -yprdeg[0]);
    inputs.headingRateDegPerSec = xyzDps[2]; // degrees per second
  }

  @Override
  public boolean isConnected() {
    return gyro.getLastError().equals(ErrorCode.OK);
  }

  @Override
  public void setGyroDirection(int direction) {
    ccwHeading = (direction == DRIVEGYRO_CCW);
  }

  @Override
  public int getGyroDirection() {
    return ccwHeading ? DRIVEGYRO_CCW : DRIVEGYRO_CW;
  }

  /**
   * Set the robot's heading offset.
   *
   * @param offsetDegrees The offset to set to, in degrees on [-180, 180].
   */
  @Override
  public void setHeadingOffset(final double offsetDegrees) {
    headingOffset = offsetDegrees;
  }

  /**
   * Get the robot's heading offset.
   *
   * @return The offset to set to, in degrees on [-180, 180].
   */
  @Override
  public double getHeadingOffset() {
    return headingOffset;
  }

  @Override
  public void calibrate() {}

  @Override
  public void close() {
    gyro.close();
  }

  /** Zero the robot's heading. */
  @Override
  public void reset() {
    gyro.reset();
    headingOffset = 0.0;
  }

  @Override
  public double getAngle() {
    return headingOffset + (ccwHeading ? gyro.getYaw() : -gyro.getYaw());
  }

  @Override
  public double getRate() {
    return gyro.getRate();
  }

  @Override
  public double getYaw() {
    return gyro.getYaw();
  }

  @Override
  public double getPitch() {
    return gyro.getPitch();
  }

  @Override
  public double getRoll() {
    return gyro.getRoll();
  }

  public void initLogging() {
    if (DEBUGGING) {
      ShuffleboardTab tabMain = Shuffleboard.getTab("GYRO");
      ShuffleboardLayout layout =
          tabMain.getLayout("Gyro", BuiltInLayouts.kList).withPosition(0, 0).withSize(4, 4);

      layout.addNumber("IMU_Yaw", () -> gyro.getYaw());
      layout.addNumber("IMU_Pitch", () -> gyro.getPitch());
      layout.addNumber("IMU_Roll", () -> gyro.getRoll());
      layout.addNumber("IMU_TotalYaw", () -> gyro.getAngle());

      /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
      layout.addNumber("IMU_CompassHeading", () -> gyro.getCompassHeading());

      /* These functions are compatible w/the WPI Gyro Class */
      layout.addNumber("IMU_TotalYaw", () -> gyro.getAngle());
      layout.addNumber("IMU_YawRateDPS", () -> gyro.getRate());
    }
  }
}
