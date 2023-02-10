package frc.lib.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.drivetrain.*;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANCoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            DriveTrainConstants.angleEnableCurrentLimit,
            RobotPreferences.Swerve.angleContinuousCurrentLimit.get(),
            RobotPreferences.Swerve.anglePeakCurrentLimit.get(),
            RobotPreferences.Swerve.anglePeakCurrentDuration.get());

    swerveAngleFXConfig.slot0.kP = RobotPreferences.Swerve.angleKP.get();
    swerveAngleFXConfig.slot0.kI = RobotPreferences.Swerve.angleKI.get();
    swerveAngleFXConfig.slot0.kD = RobotPreferences.Swerve.angleKD.get();
    swerveAngleFXConfig.slot0.kF = RobotPreferences.Swerve.angleKF.get();
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            DriveTrainConstants.driveEnableCurrentLimit,
            RobotPreferences.Swerve.driveContinuousCurrentLimit.get(),
            RobotPreferences.Swerve.drivePeakCurrentLimit.get(),
            RobotPreferences.Swerve.drivePeakCurrentDuration.get());

    swerveDriveFXConfig.slot0.kP = RobotPreferences.Swerve.driveKP.get();
    swerveDriveFXConfig.slot0.kI = RobotPreferences.Swerve.driveKI.get();
    swerveDriveFXConfig.slot0.kD = RobotPreferences.Swerve.driveKD.get();
    swerveDriveFXConfig.slot0.kF = RobotPreferences.Swerve.driveKF.get();
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.openloopRamp = RobotPreferences.Swerve.openLoopRamp.get();
    swerveDriveFXConfig.closedloopRamp = RobotPreferences.Swerve.closedLoopRamp.get();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = DriveTrainConstants.canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
