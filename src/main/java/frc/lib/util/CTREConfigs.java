package frc.lib.util;

import frc.robot.Constants;
import frc.robot.RobotPreferences;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            RobotPreferences.Swerve.angleEnableCurrentLimit(), 
            RobotPreferences.Swerve.angleContinuousCurrentLimit(), 
            RobotPreferences.Swerve.anglePeakCurrentLimit(),
            RobotPreferences.Swerve.anglePeakCurrentDuration());

        swerveAngleFXConfig.slot0.kP = RobotPreferences.Swerve.angleKP();
        swerveAngleFXConfig.slot0.kI = RobotPreferences.Swerve.angleKI();
        swerveAngleFXConfig.slot0.kD = RobotPreferences.Swerve.angleKD();
        swerveAngleFXConfig.slot0.kF = RobotPreferences.Swerve.angleKF();
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            RobotPreferences.Swerve.driveEnableCurrentLimit(), 
            RobotPreferences.Swerve.driveContinuousCurrentLimit(), 
            RobotPreferences.Swerve.drivePeakCurrentLimit(), 
            RobotPreferences.Swerve.drivePeakCurrentDuration());

        swerveDriveFXConfig.slot0.kP = RobotPreferences.Swerve.driveKP();
        swerveDriveFXConfig.slot0.kI = RobotPreferences.Swerve.driveKI();
        swerveDriveFXConfig.slot0.kD = RobotPreferences.Swerve.driveKD();
        swerveDriveFXConfig.slot0.kF = RobotPreferences.Swerve.driveKF();        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = RobotPreferences.Swerve.openLoopRamp();
        swerveDriveFXConfig.closedloopRamp = RobotPreferences.Swerve.closedLoopRamp();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}