package frc.lib.gyro;

import static frc.robot.Constants.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroIO implements Gyro {

  public static final int DRIVEGYRO_NAVX = -1; // values from 0+ are for Pigeon CAN devices
  public static final int DRIVEGYRO_CCW = 0; // set gyro yaw to ccw for angle direction
  public static final int DRIVEGYRO_CW = 1; // set gyro yaw to clockwise for angle direction

  private AHRS ahrs = null;
  private WPI_Pigeon2 pigeon = null;

  private boolean ccwHeading = false;
  private double headingOffset = 0.0;

  public GyroIO(int gyroId, String canBus) {
    if (gyroId == DRIVEGYRO_NAVX) {
      ahrs = new AHRS();
      ccwHeading = false;
    } else {
      pigeon = new WPI_Pigeon2(gyroId, canBus);
      pigeon.configFactoryDefault();
      ccwHeading = true;
    }
    initLogging();
  }

  public boolean isConnected() {
    return (((ahrs != null) && ahrs.isConnected())
        || ((pigeon != null) && pigeon.getLastError().equals(ErrorCode.OK)));
  }

  public void setGyroDirection(int direction) {
    ccwHeading = (direction == DRIVEGYRO_CCW);
  }

  public int getGyroDirection() {
    return ccwHeading ? DRIVEGYRO_CCW : DRIVEGYRO_CW;
  }

  /**
   * Set the robot's heading offset.
   *
   * @param offsetDegrees The offset to set to, in degrees on [-180, 180].
   */
  public void setHeadingOffset(final double offsetDegrees) {
    headingOffset = offsetDegrees;
  }

  /**
   * Get the robot's heading offset.
   *
   * @return The offset to set to, in degrees on [-180, 180].
   */
  public double getHeadingOffset() {
    return headingOffset;
  }

  /**
   * Set the robot's heading.
   *
   * @param heading The heading to set to, in degrees on [-180, 180].
   */
  public void setHeadingDegrees(final double heading) {
    if (ahrs != null) {
      ahrs.setAngleAdjustment(
          ccwHeading ? (360.0 - heading) + ahrs.getYaw() : heading + ahrs.getYaw());
    } else if (pigeon != null) {
      pigeon.setYaw(ccwHeading ? heading : 360.0 - heading);
    }
  }

  public void setRawHeadingDegrees(final double heading) {
    if (ahrs != null) {
      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
      angle.set(ccwHeading ? 360.0 - heading : heading);
    } else if (pigeon != null) {
      pigeon.setYaw(ccwHeading ? heading : 360.0 - heading);
    }
  }

  public void calibrate() {}

  public void close() {
    if (ahrs != null) {
      ahrs.close();
    } else if (pigeon != null) {
      pigeon.close();
    }
  }

  /** Zero the robot's heading. */
  public void reset() {
    if (ahrs != null) {
      ahrs.reset();
    }
    if (pigeon != null) {
      pigeon.reset();
    }
    headingOffset = 0.0;
  }

  public double getAngle() {
    double heading = 0.0;
    if (ahrs != null) {
      heading = ccwHeading ? 360.0 - ahrs.getAngle() : ahrs.getAngle();
    } else if (pigeon != null) {
      heading = ccwHeading ? pigeon.getYaw() : pigeon.getAngle();
    }
    return heading + headingOffset;
  }

  public double getRate() {
    if (ahrs != null) {
      return -ahrs.getRate();
    } else if (pigeon != null) {
      return pigeon.getRate();
    }
    return 0.0;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getPitch() {
    if (ahrs != null) {
      return ahrs.getPitch();
    } else if (pigeon != null) {
      return pigeon.getPitch();
    }
    return 0.0;
  }

  public double getRoll() {
    if (ahrs != null) {
      return ahrs.getRoll();
    } else if (pigeon != null) {
      return pigeon.getRoll();
    }
    return 0.0;
  }

  public void initLogging() {
    ShuffleboardTab tabMain = Shuffleboard.getTab("GYRO");

    if (ahrs != null) {
      tabMain.addBoolean("Gyro/IMU_Connected", () -> ahrs.isConnected());
      tabMain.addNumber("Gyro/IMU_Yaw", () -> ahrs.getYaw());
      tabMain.addNumber("Gyro/IMU_Pitch", () -> ahrs.getPitch());
      tabMain.addNumber("Gyro/IMU_Roll", () -> ahrs.getRoll());

      if (TESTING) {
        /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
        tabMain.addNumber("Gyro/IMU_CompassHeading", () -> ahrs.getCompassHeading());
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        tabMain.addNumber("Gyro/IMU_FusedHeading", () -> ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class */
        tabMain.addNumber("Gyro/IMU_TotalYaw", () -> ahrs.getAngle());
        tabMain.addNumber("Gyro/IMU_YawRateDPS", () -> ahrs.getRate());
      }
    } else {
      tabMain.addNumber("Gyro/IMU_Yaw", () -> pigeon.getYaw());
      tabMain.addNumber("Gyro/IMU_Pitch", () -> pigeon.getPitch());
      tabMain.addNumber("Gyro/IMU_Roll", () -> pigeon.getRoll());
      tabMain.addNumber("Gyro/IMU_TotalYaw", () -> pigeon.getAngle());

      if (TESTING) {
        /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
        tabMain.addNumber("Gyro/IMU_CompassHeading", () -> pigeon.getCompassHeading());

        /* These functions are compatible w/the WPI Gyro Class */
        tabMain.addNumber("Gyro/IMU_TotalYaw", () -> pigeon.getAngle());
        tabMain.addNumber("Gyro/IMU_YawRateDPS", () -> pigeon.getRate());
      }
    }
  }

  public void logPeriodic() {
    /* Smart dash plots */
    if (ahrs != null) {
      SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
      // SmartDashboard.putBoolean( "IMU_IsCalibrating",    ahrs.isCalibrating());
      SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
      SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
      SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

      /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
      SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

      /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
      SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

      /* These functions are compatible w/the WPI Gyro Class */
      SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
      SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

      /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
      //          SmartDashboard.putNumber(  "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
      //          SmartDashboard.putNumber(  "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
      SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
      SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

      /* Display estimates of velocity/displacement.  Note that these values are  */
      /* not expected to be accurate enough for estimating robot position on a    */
      /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
      /* of these errors due to single (velocity) integration and especially      */
      /* double (displacement) integration.                                       */
      //          SmartDashboard.putNumber(  "IMU_Temp_C",           ahrs.getTempC());
      //          SmartDashboard.putNumber(  "Velocity_X",           ahrs.getVelocityX() );
      //          SmartDashboard.putNumber(  "Velocity_Y",           ahrs.getVelocityY() );
      //          SmartDashboard.putNumber(  "Displacement_X",       ahrs.getDisplacementX() );
      //          SmartDashboard.putNumber(  "Displacement_Y",       ahrs.getDisplacementY() );

      /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
      /* NOTE:  These values are not normally necessary, but are made available   */
      /* for advanced users.  Before using this data, please consider whether     */
      /* the processed data (see above) will suit your needs.                     */
      //          SmartDashboard.putNumber(   "RawGyro_X",           ahrs.getRawGyroX());
      //          SmartDashboard.putNumber(   "RawGyro_Y",           ahrs.getRawGyroY());
      //          SmartDashboard.putNumber(   "RawGyro_Z",           ahrs.getRawGyroZ());
      //          SmartDashboard.putNumber(   "RawAccel_X",          ahrs.getRawAccelX());
      //          SmartDashboard.putNumber(   "RawAccel_Y",          ahrs.getRawAccelY());
      //          SmartDashboard.putNumber(   "RawAccel_Z",          ahrs.getRawAccelZ());
      //          SmartDashboard.putNumber(   "RawMag_X",            ahrs.getRawMagX());
      //          SmartDashboard.putNumber(   "RawMag_Y",            ahrs.getRawMagY());
      //          SmartDashboard.putNumber(   "RawMag_Z",            ahrs.getRawMagZ());
      //          SmartDashboard.putNumber(   "IMU_Temp_C",          ahrs.getTempC());

      /* Omnimount Yaw Axis Information                                           */
      /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
      //          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
      //          SmartDashboard.putString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
      //          SmartDashboard.putNumber(  "YawAxis",
      // yaw_axis.board_axis.getValue());

      /* Sensor Board Information                                                 */
      //          SmartDashboard.putString(  "FirmwareVersion",      ahrs.getFirmwareVersion());

      /* Quaternion Data                                                          */
      /* Quaternions are fascinating, and are the most compact representation of  */
      /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
      /* from the Quaternions.  If interested in motion processing, knowledge of  */
      /* Quaternions is highly recommended.                                       */
      //          SmartDashboard.putNumber(  "QuaternionW",          ahrs.getQuaternionW());
      //          SmartDashboard.putNumber(  "QuaternionX",          ahrs.getQuaternionX());
      //          SmartDashboard.putNumber(  "QuaternionY",          ahrs.getQuaternionY());
      //          SmartDashboard.putNumber(  "QuaternionZ",          ahrs.getQuaternionZ());

      /* Connectivity Debugging Support                                           */
      //          SmartDashboard.putNumber(  "IMU_Update_Count",     ahrs.getUpdateCount());
      //          SmartDashboard.putNumber(  "IMU_Byte_Count",       ahrs.getByteCount());
    }
    if (pigeon != null) {
      SmartDashboard.putNumber("IMU_Yaw", pigeon.getYaw());
      SmartDashboard.putNumber("IMU_Pitch", pigeon.getPitch());
      SmartDashboard.putNumber("IMU_Roll", pigeon.getRoll());

      /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
      SmartDashboard.putNumber("IMU_CompassHeading", pigeon.getCompassHeading());

      /* These functions are compatible w/the WPI Gyro Class */
      SmartDashboard.putNumber("IMU_TotalYaw", pigeon.getAngle());
      SmartDashboard.putNumber("IMU_YawRateDPS", pigeon.getRate());
    }
  }
}
