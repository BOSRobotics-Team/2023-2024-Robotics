package frc.robot.operator_interface;

/**
 * {@link OperatorInterface} JSON parsed class. Used to access parsed data from the
 * swervedrive.json file.
 */
public class OIJson {
  /** Robot IMU used to determine heading of the robot. */
  public DeviceJson imu;
  /** Invert the IMU of the robot. */
  public boolean invertedIMU;
  /** Module JSONs in order clockwise order starting from front left. */
  public String[] modules;
}
