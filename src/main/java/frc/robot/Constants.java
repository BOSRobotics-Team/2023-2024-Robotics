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

  public static final double STICK_DEADBAND = 0.01;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double TURN_CONSTANT = 0.75;

  public static final class VisionConstants {
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
  }

  public static final class DriveTrainConstants {
    /** Maximum Speed - Meters per Second */
    public static final double maxSpeed = 4.5;

    public static final double maxAngularVelocity = 5.0;

    /* Module Gear Ratios */
    public static final String swerveConfigurationName = "serve/neo";
    public static final double wheelDiameter = SwerveDriveConstants.wheelDiameter_MK4i;
    public static final double driveGearRatio = SwerveDriveConstants.driveGearRatio_MK4i_L2;
    public static final double angleGearRatio = SwerveDriveConstants.angleGearRatio_MK4i;

    // public static final String swerveConfigurationName = "serve/falcon";
    // public static final double wheelDiameter = SwerveDriveConstants.wheelDiameter_MK4;
    // public static final double driveGearRatio = SwerveDriveConstants.driveGearRatio_MK4_L2;
    // public static final double angleGearRatio = SwerveDriveConstants.angleGearRatio_MK4;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_ID = 21;
    public static final int LEFTSHOOTER_ID = 22;
    public static final int RIGHTSHOOTER_ID = 23;

    public static final double proportialPIDConstant = 0.0002;
    public static final double integralPIDConstant = 0.0;
    public static final double derivativePIDConstant = 0.0;
    public static final double integralPIDZone = 0.0;
    public static final double leftFeedForwardPIDConstant = 0.000175;
    public static final double rightFeedForwardPIDConstant = 0.000170;
    public static final double maxPIDOutput = 1.0;
    public static final double minPIDOutput = 0.0;
    public static final double velocityPIDTolerance = 30;
    public static final double kTargetLeftVelocity = 0;
    public static int intakeSensorID;
    public static double intakeRunSpeed;
    public static double intakeReverseSpeed;
    public static double kTargetRightVelocity;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 0.75;

    public static final double kPXController = 5.0;
    public static final double kIXController = 0.0;
    public static final double kDXController = 0.0;
    public static final double kPYController = 5.0;
    public static final double kIYController = 0.0;
    public static final double kDYController = 0.0;
    public static final double kPThetaController = 1;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0;
  }

  public static final class TestsystemConstants {

    public static final int MAX_TEST_COLUMNS = 10;
  }
}
