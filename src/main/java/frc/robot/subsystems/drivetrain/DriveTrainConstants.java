package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.swerve.SwerveModuleIO.SwerveModuleID;

public final class DriveTrainConstants {

  public static final COTSFalconSwerveConstants moduleType =
      COTSFalconSwerveConstants.SDSMK4_L2();

  public static final SwerveModuleID mod0 =
      new SwerveModuleID(
          FRONT_LEFT_MODULE,
          SWERVE_CAN_BUS,
          FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
          FRONT_LEFT_MODULE_ANGLE_MOTOR_ID,
          FRONT_LEFT_MODULE_ANGLE_ENCODER_ID,
          FRONT_LEFT_MODULE_ANGLE_OFFSET);
  public static final SwerveModuleID mod1 =
      new SwerveModuleID(
          FRONT_RIGHT_MODULE,
          SWERVE_CAN_BUS,
          FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
          FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID,
          FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID,
          FRONT_RIGHT_MODULE_ANGLE_OFFSET);
  public static final SwerveModuleID mod2 =
      new SwerveModuleID(
          BACK_LEFT_MODULE,
          SWERVE_CAN_BUS,
          BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
          BACK_LEFT_MODULE_ANGLE_MOTOR_ID,
          BACK_LEFT_MODULE_ANGLE_ENCODER_ID,
          BACK_LEFT_MODULE_ANGLE_OFFSET);
  public static final SwerveModuleID mod3 =
      new SwerveModuleID(
          BACK_RIGHT_MODULE,
          SWERVE_CAN_BUS,
          BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
          BACK_RIGHT_MODULE_ANGLE_MOTOR_ID,
          BACK_RIGHT_MODULE_ANGLE_ENCODER_ID,
          BACK_RIGHT_MODULE_ANGLE_OFFSET);

  /* Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(WHEELBASE / 2.0, TRACKWIDTH / 2.0),
          new Translation2d(WHEELBASE / 2.0, -TRACKWIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, TRACKWIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, -TRACKWIDTH / 2.0));

  public static final double wheelCircumference = moduleType.wheelCircumference;

  /* Module Gear Ratios */
  public static final double driveGearRatio = moduleType.driveGearRatio;
  public static final double angleGearRatio = moduleType.angleGearRatio;

  /* Motor Inverts */
  public static final boolean angleMotorInvert = moduleType.angleMotorInvert;
  public static final boolean driveMotorInvert = moduleType.driveMotorInvert;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = moduleType.canCoderInvert;

  /* Angle Encoder Invert */
  public static final double startAngleDegrees = 0.0;

  /* Swerve Current Limiting */
  public static final int driveContinuousCurrentLimit = 35;
  public static final int drivePeakCurrentLimit = 60;
  public static final double drivePeakCurrentDuration = 0.1;
  public static final boolean driveEnableCurrentLimit = true;

  public static final int angleContinuousCurrentLimit = 25;
  public static final int anglePeakCurrentLimit = 40;
  public static final double anglePeakCurrentDuration = 0.1;
  public static final boolean angleEnableCurrentLimit = true;

  /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKF = 0.0;

  /* Angle Motor PID Values */
  public static final double angleKP = moduleType.angleKP;
  public static final double angleKI = moduleType.angleKI;
  public static final double angleKD = moduleType.angleKD;
  public static final double angleKF = moduleType.angleKF;

  /* Drive Motor Characterization Values
   * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
  public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
  public static final double driveKV = (1.51 / 12);
  public static final double driveKA = (0.27 / 12);

  /* Swerve Profiling Values */
  /** Meters per Second */
  public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
  /** Radians per Second */
  public static final double maxAngularVelocity = 5.0; // TODO: This must be tuned to specific robot

  /* Neutral Modes */
  public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
  public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

  public static final double maxCoastVelocity_MPS =
      6380.0 / 60.0 / driveGearRatio * wheelCircumference;
}
