package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveDriveConstants;
import swervelib.math.Matter;

public final class Constants {
  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;

  public static final String LIMELIGHTNAME = "limelight";
  public static final String LIMELIGHTURL = "limelight.local";
  public static final String PHOTONVISIONURL = "photonvision.local";

  public static final String kCameraName1 = "OV5647";
  public static final String kCameraName2 = "camera2";

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  public static final Transform3d kRobotToCam1 =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /** Maximum Speed - Meters per Second */
  public static final double maxSpeed = 4.5;

  public static final double maxAngularVelocity = 5.0;

  /* Module Gear Ratios */
  public static final String swerveConfigurationName = "serve/neo";
  public static final double wheelDiameter =
      SwerveDriveConstants.wheelDiameter_MK4i; // driveGearRatio_MK4_L2;
  public static final double driveGearRatio =
      SwerveDriveConstants.driveGearRatio_MK4i_L2; // driveGearRatio_MK4_L2;
  public static final double angleGearRatio =
      SwerveDriveConstants.angleGearRatio_MK4i; // angleGearRatio_MK4;

  // public static final String swerveConfigurationName = "serve/falcon";
  // public static final double wheelDiameter = SwerveDriveConstants.wheelDiameter_MK4;
  // public static final double driveGearRatio = SwerveDriveConstants.driveGearRatio_MK4_L2;
  // public static final double angleGearRatio = SwerveDriveConstants.angleGearRatio_MK4;

  public static final double STICK_DEADBAND = 0.01;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double TURN_CONSTANT = 0.75;

  public static final int MAX_TEST_COLUMNS = 10;
}
