package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;

  public static final String GYRO_CAN_BUS = "Swerve";
  public static final int GYRO_ID = 20; // DriveGyro.DRIVEGYRO_NAVX;

  public static final String LIMELIGHTNAME = "limelight";
  public static final String LIMELIGHTURL = "limelight.local";

  public static final int PNEUMATICSHUB_ID = 29;
  public static final int SOLENOID_FWD_CHANNEL = 0;
  public static final int SOLENOID_REV_CHANNEL = 1;

  public static final int ARM_LIFT_MOTOR_ID = 30;
  public static final int ARM_EXTEND_MOTOR_ID = 31;

  public static final double STICK_DEADBAND = 0.1;

  public static final double TRACKWIDTH = Units.inchesToMeters(17.5);
  public static final double WHEELBASE = Units.inchesToMeters(25.5);
  public static final double DRIVE_BASE_RADIUS = 0.4; // Drive base radius in meters. Distance from robot center to furthest module.

  public static final String SWERVE_CAN_BUS = "Swerve";

  public static final int FRONT_LEFT_MODULE = 0;
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 3;
  public static final int FRONT_LEFT_MODULE_ANGLE_MOTOR_ID = 2;
  public static final int FRONT_LEFT_MODULE_ANGLE_ENCODER_ID = 12;
  public static final double FRONT_LEFT_MODULE_ANGLE_OFFSET = 57.920;

  public static final int FRONT_RIGHT_MODULE = 1;
  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
  public static final int FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID = 0;
  public static final int FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID = 10;
  public static final double FRONT_RIGHT_MODULE_ANGLE_OFFSET = 344.180;

  public static final int BACK_LEFT_MODULE = 2;
  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 5;
  public static final int BACK_LEFT_MODULE_ANGLE_MOTOR_ID = 4;
  public static final int BACK_LEFT_MODULE_ANGLE_ENCODER_ID = 14;
  public static final double BACK_LEFT_MODULE_ANGLE_OFFSET = 350.332;

  public static final int BACK_RIGHT_MODULE = 3;
  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 7;
  public static final int BACK_RIGHT_MODULE_ANGLE_MOTOR_ID = 6;
  public static final int BACK_RIGHT_MODULE_ANGLE_ENCODER_ID = 16;
  public static final double BACK_RIGHT_MODULE_ANGLE_OFFSET = 42.539;

  

}
