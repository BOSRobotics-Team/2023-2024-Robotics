package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

public final class RobotPreferences {

    public static double frontLeftModule_AngleOffset() { 
        return Preferences.getDouble("frontLeftModule_AngleOffset", Constants.FRONT_LEFT_MODULE_ANGLE_OFFSET); 
    }
    public static double frontRightModule_AngleOffset() { 
        return Preferences.getDouble("frontRightModule_AngleOffset", Constants.FRONT_RIGHT_MODULE_ANGLE_OFFSET); 
    }
    public static double backLeftModule_AngleOffset() { 
        return Preferences.getDouble("backLeftModule_AngleOffset", Constants.BACK_LEFT_MODULE_ANGLE_OFFSET); 
    }
    public static double backRightModule_AngleOffset() { 
        return Preferences.getDouble("backRightModule_AngleOffset", Constants.BACK_RIGHT_MODULE_ANGLE_OFFSET); 
    }
    public static double stickDeadband() { 
        return Preferences.getDouble("stickDeadband", Constants.stickDeadband); 
    }
    public static double trackWidth() { 
        return Preferences.getDouble("trackWidth", Constants.trackWidth); 
    }
    public static double wheelBase() { 
        return Preferences.getDouble("wheelBase", Constants.wheelBase); 
    }

    public static final class Swerve {
        public static int driveContinuousCurrentLimit() { 
            return Preferences.getInt("driveContinuousCurrentLimit", Constants.Swerve.driveContinuousCurrentLimit); 
        }
        public static int drivePeakCurrentLimit() { 
            return Preferences.getInt("drivePeakCurrentLimit", Constants.Swerve.drivePeakCurrentLimit); 
        }
        public static double drivePeakCurrentDuration() { 
            return Preferences.getDouble("drivePeakCurrentDuration", Constants.Swerve.drivePeakCurrentDuration); 
        }
        public static Boolean driveEnableCurrentLimit() { 
            return Preferences.getBoolean("driveEnableCurrentLimit", Constants.Swerve.driveEnableCurrentLimit); 
        }
        public static int angleContinuousCurrentLimit() { 
            return Preferences.getInt("angleContinuousCurrentLimit", Constants.Swerve.angleContinuousCurrentLimit); 
        }
        public static int anglePeakCurrentLimit() { 
            return Preferences.getInt("anglePeakCurrentLimit", Constants.Swerve.anglePeakCurrentLimit); 
        }
        public static double anglePeakCurrentDuration() { 
            return Preferences.getDouble("anglePeakCurrentDuration", Constants.Swerve.anglePeakCurrentDuration); 
        }
        public static Boolean angleEnableCurrentLimit() { 
            return Preferences.getBoolean("angleEnableCurrentLimit", Constants.Swerve.angleEnableCurrentLimit); 
        }
        public static double openLoopRamp() { 
            return Preferences.getDouble("openLoopRamp", Constants.Swerve.openLoopRamp); 
        }
        public static double closedLoopRamp() { 
            return Preferences.getDouble("closedLoopRamp", Constants.Swerve.closedLoopRamp); 
        }
        public static double driveKP() { 
            return Preferences.getDouble("driveKP", Constants.Swerve.driveKP); 
        }
        public static double driveKI() { 
            return Preferences.getDouble("driveKI", Constants.Swerve.driveKI); 
        }
        public static double driveKD() { 
            return Preferences.getDouble("driveKD", Constants.Swerve.driveKD); 
        }
        public static double driveKF() { 
            return Preferences.getDouble("driveKF", Constants.Swerve.driveKF); 
        }
        public static double angleKP() { 
            return Preferences.getDouble("angleKP", Constants.swerveModuleType.angleKP); 
        }
        public static double angleKI() { 
            return Preferences.getDouble("angleKI", Constants.swerveModuleType.angleKI); 
        }
        public static double angleKD() { 
            return Preferences.getDouble("angleKD", Constants.swerveModuleType.angleKD); 
        }
        public static double angleKF() { 
            return Preferences.getDouble("angleKF", Constants.swerveModuleType.angleKF); 
        }
        public static double driveKS() { 
            return Preferences.getDouble("driveKS", Constants.Swerve.driveKS); 
        }
        public static double driveKV() { 
            return Preferences.getDouble("driveKV", Constants.Swerve.driveKV); 
        }
        public static double driveKA() { 
            return Preferences.getDouble("driveKA", Constants.Swerve.driveKA); 
        }
        public static double maxSpeed() { 
            return Preferences.getDouble("maxSpeed", Constants.Swerve.maxSpeed); 
        }
        public static double maxAngularVelocity() { 
            return Preferences.getDouble("maxAngularVelocity", Constants.Swerve.maxAngularVelocity); 
        }
        public static double maxCoastVelocity_MPS() { 
            return Preferences.getDouble("maxCoastVelocity_MPS", Constants.Swerve.maxCoastVelocity_MPS); 
        }
    }

    public static final class AutoConstants {
        public static double maxSpeedMetersPerSecond() { 
            return Preferences.getDouble("maxSpeedMetersPerSecond", Constants.AutoConstants.kMaxSpeedMetersPerSecond); 
        }
        public static double maxAccelerationMetersPerSecondSquared() { 
            return Preferences.getDouble("maxAccelerationMetersPerSecondSquared", Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared); 
        }
        public static double maxAngularSpeedRadiansPerSecond() { 
            return Preferences.getDouble("maxAngularSpeedRadiansPerSecond", Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond); 
        }
        public static double maxAngularSpeedRadiansPerSecondSquared() { 
            return Preferences.getDouble("maxAngularSpeedRadiansPerSecondSquared", Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared); 
        }
        public static double pXController() { 
            return Preferences.getDouble("pXController", Constants.AutoConstants.kPXController); 
        }
        public static double iXController() { 
            return Preferences.getDouble("iXController", Constants.AutoConstants.kIXController); 
        }
        public static double dXController() { 
            return Preferences.getDouble("dXController", Constants.AutoConstants.kDXController); 
        }
        public static double pYController() { 
            return Preferences.getDouble("pYController", Constants.AutoConstants.kPYController); 
        }
        public static double iYController() { 
            return Preferences.getDouble("iYController", Constants.AutoConstants.kIYController); 
        }
        public static double dYController() { 
            return Preferences.getDouble("dYController", Constants.AutoConstants.kDYController); 
        }
        public static double pThetaController() { 
            return Preferences.getDouble("pThetaController", Constants.AutoConstants.kPThetaController); 
        }
        public static double iThetaController() { 
            return Preferences.getDouble("iThetaController", Constants.AutoConstants.kIThetaController); 
        }
        public static double dThetaController() { 
            return Preferences.getDouble("dThetaController", Constants.AutoConstants.kDThetaController); 
        }
    }

    public static final class ArmConstants {
        public static double armKP() { 
            return Preferences.getDouble("armKP", Constants.ArmConstants.armKP); 
        }
        public static double armKI() { 
            return Preferences.getDouble("armKI", Constants.ArmConstants.armKI); 
        }
        public static double armKD() { 
            return Preferences.getDouble("armKD", Constants.ArmConstants.armKD); 
        }
        public static double armKIZ() { 
            return Preferences.getDouble("armKIZ", Constants.ArmConstants.armKIZ); 
        }
        public static double armKFF() { 
            return Preferences.getDouble("armKFF", Constants.ArmConstants.armKFF); 
        }
        public static double armMaxOutput() { 
            return Preferences.getDouble("armMaxOutput", Constants.ArmConstants.armMaxOutput); 
        }
        public static double armMinOutput() { 
            return Preferences.getDouble("armMinOutput", Constants.ArmConstants.armMinOutput); 
        }
        public static double armMaxRPM() { 
            return Preferences.getDouble("armMaxRPM", Constants.ArmConstants.armMaxRPM); 
        }
        public static double armMaxVel() { 
            return Preferences.getDouble("armMaxVel", Constants.ArmConstants.armMaxVel); 
        }
        public static double armMinVel() { 
            return Preferences.getDouble("armMinVel", Constants.ArmConstants.armMinVel); 
        }
        public static double armMaxAcc() { 
            return Preferences.getDouble("armMaxAcc", Constants.ArmConstants.armMaxAcc); 
        }
        public static double armAllowedErr() { 
            return Preferences.getDouble("armAllowedErr", Constants.ArmConstants.armAllowedErr); 
        }
        public static double armLiftGearRatio() { 
            return Preferences.getDouble("armLiftGearRatio", Constants.ArmConstants.armLiftGearRatio); 
        }
        public static double armLiftMetersPerRotation() { 
            return Preferences.getDouble("armLiftMetersPerRotation", Constants.ArmConstants.armLiftMetersPerRotation); 
        }
        public static double armExtendGearRatio() { 
            return Preferences.getDouble("armExtendGearRatio", Constants.ArmConstants.armExtendGearRatio); 
        }
        public static double armExtendMetersPerRotation() { 
            return Preferences.getDouble("armExtendMetersPerRotation", Constants.ArmConstants.armExtendMetersPerRotation); 
        }
        public static double armLiftMinHeight() { 
            return Preferences.getDouble("armLiftMinHeight", Constants.ArmConstants.armLiftMinHeight); 
        }
        public static double armLiftMaxHeight() { 
            return Preferences.getDouble("armLiftMaxHeight", Constants.ArmConstants.armLiftMaxHeight); 
        }
        public static double armExtendMinLength() { 
            return Preferences.getDouble("armExtendMinLength", Constants.ArmConstants.armExtendMinLength); 
        }
        public static double armExtendMaxLength() { 
            return Preferences.getDouble("armExtendMaxLength", Constants.ArmConstants.armExtendMaxLength); 
        }
    }
}
