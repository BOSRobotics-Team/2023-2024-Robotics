package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.DriveGyro;

public final class Constants {

    public static final String SWERVE_CAN_BUS = "Swerve";

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 8.262;
  
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 45.439;
  
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 96.592;
  
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 13.799;

    public static final String GYRO_CAN_BUS = "";
    public static final int GYRO_ID = DriveGyro.DRIVEGYRO_NAVX;
    public static final int GYRO_DIRECTION = DriveGyro.DRIVEGYRO_CCW; // Always ensure Gyro is CCW+ CW-

    public static final int ARM_LIFT_MOTOR = 0;
    public static final int ARM_EXTEND_MOTOR = 1;


    public static final double stickDeadband = 0.1;

    public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(10.75 /*21.73*/); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(13.0 /*21.73*/); //TODO: This must be tuned to specific robot

    public static final class Swerve {
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

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
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0; //10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        public static final double maxCoastVelocity_MPS = 6380.0 / 60.0 / driveGearRatio * wheelCircumference;
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kIXController = 0;
        public static final double kDXController = 0;
        public static final double kPYController = 1;
        public static final double kIYController = 0;
        public static final double kDYController = 0;
        public static final double kPThetaController = 1;    
        public static final double kIThetaController = 0;    
        public static final double kDThetaController = 0;    
    }

    public static final class ArmConstants {
         // PID coefficients
        public static final double armKP = 5e-5; 
        public static final double armKI = 1e-6;
        public static final double armKD = 0.0;
        public static final double armKIZ = 0.0;
        public static final double armKFF = 0.000156; 
        public static final double armMaxOutput = 1.0; 
        public static final double armMinOutput = -1.0; 
        public static final double armMaxRPM = 5700.0;

        // Smart Motion Coefficients
        public static final double armMaxVel = 2000.0; 
        public static final double armMinVel = 50.0; 
        public static final double armMaxAcc = 1500.0; 
        public static final double armAllowedErr = 0.0;
        public static final double armLiftGearRatio = 1.0;
        public static final double armLiftMetersPerRotation = 0.1;
        public static final double armExtendGearRatio = 1.0;
        public static final double armExtendMetersPerRotation = 0.1;
        public static final double armLiftMinHeight = 0.0;
        public static final double armLiftMaxHeight = 2.0;
        public static final double armExtendMinLength = 0.0;
        public static final double armExtendMaxLength = 2.0;
    }
}
