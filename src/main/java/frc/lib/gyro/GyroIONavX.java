package frc.lib.gyro;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;

public class GyroIONavX implements GyroIO {
  private AHRS gyro = null;

  public GyroIONavX() {
    gyro = new AHRS();
    initLogging();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = this.isConnected();
    inputs.positionDeg = 360.0 - gyro.getYaw(); // degrees
    inputs.yawDeg = gyro.getYaw(); // degrees
    inputs.yawDegPerSec = -gyro.getRate(); // degrees per second
    inputs.pitchDeg = gyro.getPitch(); // degrees
    inputs.pitchDegPerSec = gyro.getVelocityY(); // degrees per second
    inputs.rollDeg = gyro.getRoll(); // degrees
    inputs.rollDegPerSec = gyro.getVelocityX(); // degrees per second
  }

  @Override
  public boolean isConnected() {
    return gyro.isConnected();
  }

  @Override
  public void setYaw(double yaw) {
    gyro.setAngleAdjustment(360.0 - gyro.getYaw() + yaw);
  }

  @Override
  public void addYaw(double yaw) {
    if (Robot.isSimulation()) {
    gyro.setAngleAdjustment(360.0 - yaw);
    }
  }


  public void initLogging() {
    if (DEBUGGING) {
      ShuffleboardTab tabMain = Shuffleboard.getTab("GYRO");
      ShuffleboardLayout layout =
          tabMain.getLayout("Gyro", BuiltInLayouts.kList).withPosition(0, 0).withSize(4, 4);

      layout.addNumber("IMU_TotalYaw", () -> gyro.getAngle());
      layout.addNumber("IMU_YawRateDPS", () -> gyro.getRate());

      layout.addNumber("IMU_Yaw", () -> gyro.getYaw());
      layout.addNumber("IMU_Pitch", () -> gyro.getPitch());
      layout.addNumber("IMU_Roll", () -> gyro.getRoll());

      layout.addBoolean("IMU_Connected", () -> gyro.isConnected());

      /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
      layout.addNumber("IMU_CompassHeading", () -> gyro.getCompassHeading());
      /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
      layout.addNumber("IMU_FusedHeading", () -> gyro.getFusedHeading());
    }
  }

  public void logPeriodic() {
    // Smart dash plots
    // SmartDashboard.putBoolean("IMU_Connected", gyro.isConnected());
    // // SmartDashboard.putBoolean( "IMU_IsCalibrating",    gyro.isCalibrating());
    // SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
    // SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
    // SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires magnetometer calibration to be useful)                                   */
    // SmartDashboard.putNumber("IMU_CompassHeading", gyro.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    // SmartDashboard.putNumber("IMU_FusedHeading", gyro.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class */
    // SmartDashboard.putNumber("IMU_TotalYaw", gyro.getAngle());
    // SmartDashboard.putNumber("IMU_YawRateDPS", gyro.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    //          SmartDashboard.putNumber(  "IMU_Accel_X",          gyro.getWorldLinearAccelX());
    //          SmartDashboard.putNumber(  "IMU_Accel_Y",          gyro.getWorldLinearAccelY());
    // SmartDashboard.putBoolean("IMU_IsMoving", gyro.isMoving());
    // SmartDashboard.putBoolean("IMU_IsRotating", gyro.isRotating());

    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */
    //          SmartDashboard.putNumber(  "IMU_Temp_C",           gyro.getTempC());
    //          SmartDashboard.putNumber(  "Velocity_X",           gyro.getVelocityX() );
    //          SmartDashboard.putNumber(  "Velocity_Y",           gyro.getVelocityY() );
    //          SmartDashboard.putNumber(  "Displacement_X",       gyro.getDisplacementX() );
    //          SmartDashboard.putNumber(  "Displacement_Y",       gyro.getDisplacementY() );

    /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    /* NOTE:  These values are not normally necessary, but are made available   */
    /* for advanced users.  Before using this data, please consider whether     */
    /* the processed data (see above) will suit your needs.                     */
    //          SmartDashboard.putNumber(   "RawGyro_X",           gyro.getRawGyroX());
    //          SmartDashboard.putNumber(   "RawGyro_Y",           gyro.getRawGyroY());
    //          SmartDashboard.putNumber(   "RawGyro_Z",           gyro.getRawGyroZ());
    //          SmartDashboard.putNumber(   "RawAccel_X",          gyro.getRawAccelX());
    //          SmartDashboard.putNumber(   "RawAccel_Y",          gyro.getRawAccelY());
    //          SmartDashboard.putNumber(   "RawAccel_Z",          gyro.getRawAccelZ());
    //          SmartDashboard.putNumber(   "RawMag_X",            gyro.getRawMagX());
    //          SmartDashboard.putNumber(   "RawMag_Y",            gyro.getRawMagY());
    //          SmartDashboard.putNumber(   "RawMag_Z",            gyro.getRawMagZ());
    //          SmartDashboard.putNumber(   "IMU_Temp_C",          gyro.getTempC());

    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    //          AHRS.BoardYawAxis yaw_axis = gyro.getBoardYawAxis();
    //          SmartDashboard.putString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
    //          SmartDashboard.putNumber(  "YawAxis",
    // yaw_axis.board_axis.getValue());

    /* Sensor Board Information                                                 */
    //          SmartDashboard.putString(  "FirmwareVersion",      gyro.getFirmwareVersion());

    /* Quaternion Data                                                          */
    /* Quaternions are fascinating, and are the most compact representation of  */
    /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    /* from the Quaternions.  If interested in motion processing, knowledge of  */
    /* Quaternions is highly recommended.                                       */
    //          SmartDashboard.putNumber(  "QuaternionW",          gyro.getQuaternionW());
    //          SmartDashboard.putNumber(  "QuaternionX",          gyro.getQuaternionX());
    //          SmartDashboard.putNumber(  "QuaternionY",          gyro.getQuaternionY());
    //          SmartDashboard.putNumber(  "QuaternionZ",          gyro.getQuaternionZ());

    /* Connectivity Debugging Support                                           */
    //          SmartDashboard.putNumber(  "IMU_Update_Count",     gyro.getUpdateCount());
    //          SmartDashboard.putNumber(  "IMU_Byte_Count",       gyro.getByteCount());
  }
}
