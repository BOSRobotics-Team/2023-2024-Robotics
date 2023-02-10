package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import frc.lib.util.PreferencesValue;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;

public final class RobotPreferences {

  public static PreferencesValue stickDeadband =
      new PreferencesValue("stickDeadband", Constants.STICK_DEADBAND);
  public static PreferencesValue trackWidth =
      new PreferencesValue("trackWidth", Constants.TRACKWIDTH);
  public static PreferencesValue wheelBase = new PreferencesValue("wheelBase", Constants.WHEELBASE);

  public static final class Swerve {
    public static PreferencesValue driveContinuousCurrentLimit =
        new PreferencesValue(
            "swerve/driveContinuousCurrentLimit", DriveTrainConstants.driveContinuousCurrentLimit);
    public static PreferencesValue drivePeakCurrentLimit =
        new PreferencesValue(
            "swerve/drivePeakCurrentLimit", DriveTrainConstants.drivePeakCurrentLimit);
    public static PreferencesValue drivePeakCurrentDuration =
        new PreferencesValue(
            "swerve/drivePeakCurrentDuration", DriveTrainConstants.drivePeakCurrentDuration);
    public static PreferencesValue angleContinuousCurrentLimit =
        new PreferencesValue(
            "swerve/angleContinuousCurrentLimit", DriveTrainConstants.angleContinuousCurrentLimit);
    public static PreferencesValue anglePeakCurrentLimit =
        new PreferencesValue(
            "swerve/anglePeakCurrentLimit", DriveTrainConstants.anglePeakCurrentLimit);
    public static PreferencesValue anglePeakCurrentDuration =
        new PreferencesValue(
            "swerve/anglePeakCurrentDuration", DriveTrainConstants.anglePeakCurrentDuration);
    public static PreferencesValue openLoopRamp =
        new PreferencesValue("swerve/openLoopRamp", DriveTrainConstants.openLoopRamp);
    public static PreferencesValue closedLoopRamp =
        new PreferencesValue("swerve/closedLoopRamp", DriveTrainConstants.closedLoopRamp);
    public static PreferencesValue driveKP =
        new PreferencesValue("swerve/driveKP", DriveTrainConstants.driveKP);
    public static PreferencesValue driveKI =
        new PreferencesValue("swerve/driveKI", DriveTrainConstants.driveKI);
    public static PreferencesValue driveKD =
        new PreferencesValue("swerve/driveKD", DriveTrainConstants.driveKD);
    public static PreferencesValue driveKF =
        new PreferencesValue("swerve/driveKF", DriveTrainConstants.driveKF);
    public static PreferencesValue angleKP =
        new PreferencesValue("swerve/angleKP", DriveTrainConstants.moduleType.angleKP);
    public static PreferencesValue angleKI =
        new PreferencesValue("swerve/angleKI", DriveTrainConstants.moduleType.angleKI);
    public static PreferencesValue angleKD =
        new PreferencesValue("swerve/angleKD", DriveTrainConstants.moduleType.angleKD);
    public static PreferencesValue angleKF =
        new PreferencesValue("swerve/angleKF", DriveTrainConstants.moduleType.angleKF);
    public static PreferencesValue driveKS =
        new PreferencesValue("swerve/driveKS", DriveTrainConstants.driveKS);
    public static PreferencesValue driveKV =
        new PreferencesValue("swerve/driveKV", DriveTrainConstants.driveKV);
    public static PreferencesValue driveKA =
        new PreferencesValue("swerve/driveKA", DriveTrainConstants.driveKA);
    public static PreferencesValue maxSpeed =
        new PreferencesValue("swerve/maxSpeed", DriveTrainConstants.maxSpeed);
    public static PreferencesValue maxAngularVelocity =
        new PreferencesValue("swerve/maxAngularVelocity", DriveTrainConstants.maxAngularVelocity);
    public static PreferencesValue maxCoastVelocity_MPS =
        new PreferencesValue(
            "swerve/maxCoastVelocity_MPS", DriveTrainConstants.maxCoastVelocity_MPS);
  }

  public static final class Auto {
    public static PreferencesValue maxSpeedMetersPerSecond =
        new PreferencesValue(
            "auto/maxSpeedMetersPerSecond", AutoConstants.kMaxSpeedMetersPerSecond);
    public static PreferencesValue maxAccelerationMetersPerSecondSquared =
        new PreferencesValue(
            "auto/maxAccelerationMetersPerSecondSquared",
            AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    public static PreferencesValue maxAngularSpeedRadiansPerSecond =
        new PreferencesValue(
            "auto/maxAngularSpeedRadiansPerSecond", AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    public static PreferencesValue maxAngularSpeedRadiansPerSecondSquared =
        new PreferencesValue(
            "auto/maxAngularSpeedRadiansPerSecondSquared",
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    public static PreferencesValue pXController =
        new PreferencesValue("auto/pXController", AutoConstants.kPXController);
    public static PreferencesValue iXController =
        new PreferencesValue("auto/iXController", AutoConstants.kIXController);
    public static PreferencesValue dXController =
        new PreferencesValue("auto/dXController", AutoConstants.kDXController);
    public static PreferencesValue pYController =
        new PreferencesValue("auto/pYController", AutoConstants.kPYController);
    public static PreferencesValue iYController =
        new PreferencesValue("auto/iYController", AutoConstants.kIYController);
    public static PreferencesValue dYController =
        new PreferencesValue("auto/dYController", AutoConstants.kDYController);
    public static PreferencesValue pThetaController =
        new PreferencesValue("auto/pThetaController", AutoConstants.kPThetaController);
    public static PreferencesValue iThetaController =
        new PreferencesValue("auto/iThetaController", AutoConstants.kIThetaController);
    public static PreferencesValue dThetaController =
        new PreferencesValue("auto/dThetaController", AutoConstants.kDThetaController);
  }

  public static final class ArmLift {
    public static PreferencesValue armKP =
        new PreferencesValue("armLift/armKP", ArmConstants.armLiftKP);
    public static PreferencesValue armKI =
        new PreferencesValue("armLift/armKI", ArmConstants.armLiftKI);
    public static PreferencesValue armKD =
        new PreferencesValue("armLift/armKD", ArmConstants.armLiftKD);
    public static PreferencesValue armKIZ =
        new PreferencesValue("armLift/armKIZ", ArmConstants.armLiftKIZ);
    public static PreferencesValue armKFF =
        new PreferencesValue("armLift/armKFF", ArmConstants.armLiftKFF);
    public static PreferencesValue armMaxOutput =
        new PreferencesValue("armLift/armMaxOutput", ArmConstants.armLiftMaxOutput);
    public static PreferencesValue armMinOutput =
        new PreferencesValue("armLift/armMinOutput", ArmConstants.armLiftMinOutput);
    public static PreferencesValue armMaxRPM =
        new PreferencesValue("armLift/armMaxRPM", ArmConstants.armLiftMaxRPM);
    public static PreferencesValue armMinPosition =
        new PreferencesValue("armLift/armMinPosition", ArmConstants.armLiftMinPosition);
    public static PreferencesValue armMaxPosition =
        new PreferencesValue("armLift/armMaxPosition", ArmConstants.armLiftMaxPosition);
    public static PreferencesValue armPosition0 =
        new PreferencesValue("armLift/armPosition0", ArmConstants.armLiftPosition0);
    public static PreferencesValue armPosition1 =
        new PreferencesValue("armLift/armPosition1", ArmConstants.armLiftPosition1);
    public static PreferencesValue armPosition2 =
        new PreferencesValue("armLift/armPosition2", ArmConstants.armLiftPosition2);

    public static String liftProfileStr() {
      if (!Preferences.containsKey("armLift/liftProfileStr")) {
        Preferences.setString("armLift/liftProfileStr", ArmConstants.armLiftProfile);
      }
      return Preferences.getString("armLift/liftProfileStr", ArmConstants.armLiftProfile);
    }
  }

  public static final class ArmExtend {
    public static PreferencesValue armKP =
        new PreferencesValue("armExtend/armKP", ArmConstants.armExtendKP);
    public static PreferencesValue armKI =
        new PreferencesValue("armExtend/armKI", ArmConstants.armExtendKI);
    public static PreferencesValue armKD =
        new PreferencesValue("armExtend/armKD", ArmConstants.armExtendKD);
    public static PreferencesValue armKIZ =
        new PreferencesValue("armExtend/armKIZ", ArmConstants.armExtendKIZ);
    public static PreferencesValue armKFF =
        new PreferencesValue("armExtend/armKFF", ArmConstants.armExtendKFF);
    public static PreferencesValue armMaxOutput =
        new PreferencesValue("armExtend/armMaxOutput", ArmConstants.armExtendMaxOutput);
    public static PreferencesValue armMinOutput =
        new PreferencesValue("armExtend/armMinOutput", ArmConstants.armExtendMinOutput);
    public static PreferencesValue armMaxRPM =
        new PreferencesValue("armExtend/armMaxRPM", ArmConstants.armExtendMaxRPM);
    public static PreferencesValue armMinPosition =
        new PreferencesValue("armExtend/armMinPosition", ArmConstants.armExtendMinPosition);
    public static PreferencesValue armMaxPosition =
        new PreferencesValue("armExtend/armMaxPosition", ArmConstants.armExtendMaxPosition);
    public static PreferencesValue armPosition0 =
        new PreferencesValue("armExtend/armPosition0", ArmConstants.armExtendPosition0);
    public static PreferencesValue armPosition1 =
        new PreferencesValue("armExtend/armPosition1", ArmConstants.armExtendPosition1);
    public static PreferencesValue armPosition2 =
        new PreferencesValue("armExtend/armPosition2", ArmConstants.armExtendPosition2);
  }
}
