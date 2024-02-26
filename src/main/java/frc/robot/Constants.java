package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
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

  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = false;

  public static final double TRIGGER_DEADBAND = 0.01;
  public static final double STICK_DEADBAND = 0.01;
  public static final double TRIGGER_SPEEDFACTOR = 0.5;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double TURN_CONSTANT = 0.75;

  public static final class VisionConstants {

    public static final String LIMELIGHTNAME = "limelight";
    public static final String LIMELIGHTURL = "limelight.local";
    public static final String PHOTONVISIONURL = "photonvision.local";

    public static final String kCameraName1 = "OV9281";
    public static final String kCameraName2 = "camera2";

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam1 =
        new Transform3d(new Translation3d(0.0, -0.33, 0.4), new Rotation3d(0, 0, Math.PI));

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

    public static final String swerveConfigurationName = "swerve/neo";
    public static final SwerveDriveConstants swerveConfig = SwerveDriveConstants.SDSMK4i_L2();

    // public static final String swerveConfigurationName = "swerve/falcon";
    // public static final SwerveDriveConstants swerveConfig = SwerveDriveConstants.SDSMK4_L2();
  }

  public static final class IntakeConstants {
    public static final int INTAKEMOTOR_ID = 21;
    public static final double intakeRunSpeed = 0.2;
    public static final double intakeReverseSpeed = -0.2;
  }

  public static final class ShooterConstants {
    public static final int TOPSHOOTERMOTOR_ID = 23;
    public static final int BOTTOMSHOOTERMOTOR_ID = 22;
    public static final int WRISTMOTOR_ID = 98; // Figure it out

    public static final double kTargetVelocity = 3200.0;
    public static final double kTargetVelocity2 = 1500.0;
    public static final double shooterReverseSpeed = -0.2;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    public static final double KSConstant = 0.0; // Static feedforward gain
    public static final double proportialPIDConstant =
        0.11; // An error of 1 rotation per second results in 2V output
    public static final double integralPIDConstant =
        0.5; // An error of 1 rotation per second increases output by 0.5V every second
    public static final double derivativePIDConstant =
        0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    public static final double feedForwardPIDConstant =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    public static final double peakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -8.0; // Peak output of 8 volts
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    public static final double proportialTorquePIDConstant =
        5.0; // An error of 1 rotation per second results in 5 amps output
    public static final double integralTorquePIDConstant =
        0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    public static final double derivativeTorquePIDConstant =
        0.001; // A change of 1000 rotation per second squared results in 1 amp output
    public static final double peakForwardTorqueCurrent = 40.0; // Peak output of 40 amps
    public static final double peakReverseTorqueCurrent = -40.0; // Peak output of 40 amps

    // public static final double proportialPIDConstant = 0.00012;
    // public static final double integralPIDConstant = 0.0;
    // public static final double derivativePIDConstant = 0.001;
    // public static final double integralPIDZone = 0.0;
    // public static final double topFeedForwardPIDConstant = 0.00017;
    // public static final double bottomFeedForwardPIDConstant = 0.00017;
    // public static final double maxPIDOutput = 1.0;
    // public static final double minPIDOutput = 0.0;
    public static final double velocityTolerance = 50.0;
    public static final double topMotorKS = 0.0;
    public static final double topMotorKV = 0.0;
    public static final double topMotorKP = 0.0;
    public static final double topMotorKI = 0.0;
    public static final double topMotorKD = 0.0;
    public static final double bottomMotorKS = 0.0;
    public static final double bottomMotorKV = 0.0;
    public static final double bottomMotorKP = 0.0;
    public static final double bottomMotorKI = 0.0;
    public static final double bottomMotorKD = 0.0;
  }

  public static final class WristConstants {
    public static final double wristMotorKS = 0.0;
    public static final double wristMotorKV = 0.0;
    public static final double wristMotorKP = 0.0;
    public static final double wristMotorKI = 0.0;
    public static final double wristMotorKD = 0.0;
    public static final double kTargetWristVelocity = 0.0;
    public static final double wristReverseSpeed = 0.0;
  }

  // public static final class ClimberConstants {

  // public static final int LEFTCLIMBER_ID = 40;
  // public static final int RIGHTCLIMBER_ID = 41;
  // public static final float kLClimberMaxHeight = 335.0f;
  // public static final float kRClimberMaxHeight = 335.0f;

  // public static final double kClimberGearRatio = 1.0;
  // }

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants kPIDTranslation = new PIDConstants(0.7, 0.0, 0.0);
    public static final PIDConstants kPIDRotation = new PIDConstants(0.4, 0.0, 0.01);
  }

  public static final class TestsystemConstants {

    public static final int MAX_TEST_COLUMNS = 10;
  }
}
