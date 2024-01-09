package frc.lib.gyro;

import static frc.robot.Constants.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;

public class GyroIOPigeon2 implements GyroIO {
  private final WPI_Pigeon2 gyro;
  private final double[] xyzDps = new double[3];
  private final double[] yprdeg = new double[3];

  public GyroIOPigeon2(int id, String canBus) {
    gyro = new WPI_Pigeon2(id, canBus);
    gyro.configFactoryDefault();

    initLogging();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    gyro.getRawGyro(xyzDps);
    gyro.getYawPitchRoll(yprdeg);

    inputs.connected = this.isConnected();
    inputs.positionDeg = yprdeg[0];
    inputs.yawDeg = yprdeg[0]; // degrees
    inputs.yawDegPerSec = xyzDps[2]; // degrees per second
    inputs.pitchDeg = yprdeg[1]; // degrees
    inputs.pitchDegPerSec = xyzDps[1]; // degrees per second
    inputs.rollDeg = yprdeg[2]; // degrees
    inputs.rollDegPerSec = xyzDps[0]; // degrees per second
  }

  @Override
  public boolean isConnected() {
    if (Robot.isReal()) {
      return gyro.getLastError().equals(ErrorCode.OK);
    }
    return false;
  }

  @Override
  public void setYaw(double yaw) {
    gyro.setYaw(yaw);
  }

  @Override
  public void addYaw(double yaw) {
    if (Robot.isSimulation()) {
      gyro.addYaw(yaw);
    }
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
