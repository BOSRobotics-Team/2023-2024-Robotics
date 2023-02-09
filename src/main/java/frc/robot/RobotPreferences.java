package frc.robot;

import frc.robot.subsystems.drivetrain.DriveTrainConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.lib.util.DashboardNumber;

public final class RobotPreferences {

    public static DashboardNumber stickDeadband = 
       new DashboardNumber("stickDeadband", Constants.STICK_DEADBAND);
    public static DashboardNumber trackWidth = 
       new DashboardNumber("trackWidth", Constants.TRACKWIDTH);
    public static DashboardNumber wheelBase = 
       new DashboardNumber("wheelBase", Constants.WHEELBASE);

    public static final class Swerve {
        public static DashboardNumber driveContinuousCurrentLimit = 
        new DashboardNumber("swerve/driveContinuousCurrentLimit", DriveTrainConstants.driveContinuousCurrentLimit); 
        public static DashboardNumber drivePeakCurrentLimit = 
        new DashboardNumber("swerve/drivePeakCurrentLimit", DriveTrainConstants.drivePeakCurrentLimit); 
        public static DashboardNumber drivePeakCurrentDuration = 
           new DashboardNumber("swerve/drivePeakCurrentDuration", DriveTrainConstants.drivePeakCurrentDuration); 
        public static DashboardNumber angleContinuousCurrentLimit = 
            new DashboardNumber("swerve/angleContinuousCurrentLimit", DriveTrainConstants.angleContinuousCurrentLimit); 
        public static DashboardNumber anglePeakCurrentLimit = 
            new DashboardNumber("swerve/anglePeakCurrentLimit", DriveTrainConstants.anglePeakCurrentLimit);
        public static DashboardNumber anglePeakCurrentDuration = 
           new DashboardNumber("swerve/anglePeakCurrentDuration", DriveTrainConstants.anglePeakCurrentDuration);
        public static DashboardNumber openLoopRamp = 
           new DashboardNumber("swerve/openLoopRamp", DriveTrainConstants.openLoopRamp);
        public static DashboardNumber closedLoopRamp = 
           new DashboardNumber("swerve/closedLoopRamp", DriveTrainConstants.closedLoopRamp);
        public static DashboardNumber driveKP = 
           new DashboardNumber("swerve/driveKP", DriveTrainConstants.driveKP);
        public static DashboardNumber driveKI = 
           new DashboardNumber("swerve/driveKI", DriveTrainConstants.driveKI);
        public static DashboardNumber driveKD = 
           new DashboardNumber("swerve/driveKD", DriveTrainConstants.driveKD);
        public static DashboardNumber driveKF = 
           new DashboardNumber("swerve/driveKF", DriveTrainConstants.driveKF);
        public static DashboardNumber angleKP = 
           new DashboardNumber("swerve/angleKP", DriveTrainConstants.moduleType.angleKP);
        public static DashboardNumber angleKI = 
           new DashboardNumber("swerve/angleKI", DriveTrainConstants.moduleType.angleKI);
        public static DashboardNumber angleKD = 
           new DashboardNumber("swerve/angleKD", DriveTrainConstants.moduleType.angleKD);
        public static DashboardNumber angleKF = 
           new DashboardNumber("swerve/angleKF", DriveTrainConstants.moduleType.angleKF);
        public static DashboardNumber driveKS = 
           new DashboardNumber("swerve/driveKS", DriveTrainConstants.driveKS);
        public static DashboardNumber driveKV = 
           new DashboardNumber("swerve/driveKV", DriveTrainConstants.driveKV);
        public static DashboardNumber driveKA = 
           new DashboardNumber("swerve/driveKA", DriveTrainConstants.driveKA);
        public static DashboardNumber maxSpeed = 
           new DashboardNumber("swerve/maxSpeed", DriveTrainConstants.maxSpeed);
        public static DashboardNumber maxAngularVelocity = 
           new DashboardNumber("swerve/maxAngularVelocity", DriveTrainConstants.maxAngularVelocity);
        public static DashboardNumber maxCoastVelocity_MPS = 
           new DashboardNumber("swerve/maxCoastVelocity_MPS", DriveTrainConstants.maxCoastVelocity_MPS);
    }

    public static final class Auto {
        public static DashboardNumber maxSpeedMetersPerSecond = 
           new DashboardNumber("auto/maxSpeedMetersPerSecond", AutoConstants.kMaxSpeedMetersPerSecond);
        public static DashboardNumber maxAccelerationMetersPerSecondSquared = 
           new DashboardNumber("auto/maxAccelerationMetersPerSecondSquared", AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        public static DashboardNumber maxAngularSpeedRadiansPerSecond = 
           new DashboardNumber("auto/maxAngularSpeedRadiansPerSecond", AutoConstants.kMaxAngularSpeedRadiansPerSecond);
        public static DashboardNumber maxAngularSpeedRadiansPerSecondSquared = 
           new DashboardNumber("auto/maxAngularSpeedRadiansPerSecondSquared", AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
        public static DashboardNumber pXController = 
           new DashboardNumber("auto/pXController", AutoConstants.kPXController);
        public static DashboardNumber iXController = 
           new DashboardNumber("auto/iXController", AutoConstants.kIXController);
        public static DashboardNumber dXController = 
           new DashboardNumber("auto/dXController", AutoConstants.kDXController);
        public static DashboardNumber pYController = 
           new DashboardNumber("auto/pYController", AutoConstants.kPYController);
        public static DashboardNumber iYController = 
           new DashboardNumber("auto/iYController", AutoConstants.kIYController);
        public static DashboardNumber dYController = 
           new DashboardNumber("auto/dYController", AutoConstants.kDYController);
        public static DashboardNumber pThetaController = 
           new DashboardNumber("auto/pThetaController", AutoConstants.kPThetaController);
        public static DashboardNumber iThetaController = 
           new DashboardNumber("auto/iThetaController", AutoConstants.kIThetaController);
        public static DashboardNumber dThetaController = 
           new DashboardNumber("auto/dThetaController", AutoConstants.kDThetaController);
    }

   public static final class ArmLift {
      public static DashboardNumber armKP = 
         new DashboardNumber("armLift/armKP", ArmConstants.armLiftKP);
      public static DashboardNumber armKI = 
         new DashboardNumber("armLift/armKI", ArmConstants.armLiftKI);
      public static DashboardNumber armKD = 
         new DashboardNumber("armLift/armKD", ArmConstants.armLiftKD);
      public static DashboardNumber armKIZ = 
         new DashboardNumber("armLift/armKIZ", ArmConstants.armLiftKIZ);
      public static DashboardNumber armKFF = 
         new DashboardNumber("armLift/armKFF", ArmConstants.armLiftKFF);
      public static DashboardNumber armMaxOutput = 
         new DashboardNumber("armLift/armMaxOutput", ArmConstants.armLiftMaxOutput);
      public static DashboardNumber armMinOutput = 
         new DashboardNumber("armLift/armMinOutput", ArmConstants.armLiftMinOutput);
      public static DashboardNumber armMaxRPM = 
         new DashboardNumber("armLift/armMaxRPM", ArmConstants.armLiftMaxRPM);
      public static DashboardNumber armMinPosition = 
         new DashboardNumber("armLift/armMinPosition", ArmConstants.armLiftMinPosition);
      public static DashboardNumber armMaxPosition = 
         new DashboardNumber("armLift/armMaxPosition", ArmConstants.armLiftMaxPosition);
      public static DashboardNumber armPosition0 = 
         new DashboardNumber("armLift/armPosition0", ArmConstants.armLiftPosition0);
      public static DashboardNumber armPosition1 = 
         new DashboardNumber("armLift/armPosition1", ArmConstants.armLiftPosition1);
      public static DashboardNumber armPosition2 = 
         new DashboardNumber("armLift/armPosition2", ArmConstants.armLiftPosition2);

      //   public static DashboardNumber armMaxVel = 
      //      new DashboardNumber("armLift/armMaxVel", ArmConstants.armLiftMaxVel);
      //   public static DashboardNumber armMinVel = 
      //      new DashboardNumber("armLift/armMinVel", ArmConstants.armLiftMinVel);
      //   public static DashboardNumber armMaxAcc = 
      //      new DashboardNumber("armLift/armMaxAcc", ArmConstants.armLiftMaxAcc);
      //   public static DashboardNumber armAllowedErr = 
      //      new DashboardNumber("armLift/armAllowedErr", ArmConstants.armLiftAllowedErr);
      //   public static DashboardNumber armGearRatio = 
      //      new DashboardNumber("armLift/armLiftGearRatio", ArmConstants.armLiftGearRatio);
      //   public static DashboardNumber armMetersPerRotation = 
      //      new DashboardNumber("armLift/armLiftMetersPerRotation", ArmConstants.armLiftMetersPerRotation);
      //   public static DashboardNumber armMinHeight = 
      //      new DashboardNumber("armLift/armLiftMinHeight", ArmConstants.armLiftMinHeight);
      //   public static DashboardNumber armMaxHeight = 
      //      new DashboardNumber("armLift/armLiftMaxHeight", ArmConstants.armLiftMaxHeight);
    }
    public static final class ArmExtend {
      public static DashboardNumber armKP = 
         new DashboardNumber("armExtend/armKP", ArmConstants.armExtendKP);
      public static DashboardNumber armKI = 
         new DashboardNumber("armExtend/armKI", ArmConstants.armExtendKI);
      public static DashboardNumber armKD = 
         new DashboardNumber("armExtend/armKD", ArmConstants.armExtendKD);
      public static DashboardNumber armKIZ = 
         new DashboardNumber("armExtend/armKIZ", ArmConstants.armExtendKIZ);
      public static DashboardNumber armKFF = 
         new DashboardNumber("armExtend/armKFF", ArmConstants.armExtendKFF);
      public static DashboardNumber armMaxOutput = 
         new DashboardNumber("armExtend/armMaxOutput", ArmConstants.armExtendMaxOutput);
      public static DashboardNumber armMinOutput = 
         new DashboardNumber("armExtend/armMinOutput", ArmConstants.armExtendMinOutput);
      public static DashboardNumber armMaxRPM = 
         new DashboardNumber("armExtend/armMaxRPM", ArmConstants.armExtendMaxRPM);
      public static DashboardNumber armMinPosition = 
         new DashboardNumber("armExtend/armMinPosition", ArmConstants.armExtendMinPosition);
      public static DashboardNumber armMaxPosition = 
         new DashboardNumber("armExtend/armMaxPosition", ArmConstants.armExtendMaxPosition);
      public static DashboardNumber armPosition0 = 
         new DashboardNumber("armExtend/armPosition0", ArmConstants.armExtendPosition0);
      public static DashboardNumber armPosition1 = 
         new DashboardNumber("armExtend/armPosition1", ArmConstants.armExtendPosition1);
      public static DashboardNumber armPosition2 = 
         new DashboardNumber("armExtend/armPosition2", ArmConstants.armExtendPosition2);


      // public static DashboardNumber armMaxVel = 
      //    new DashboardNumber("armExtend/armMaxVel", ArmConstants.armExtendMaxVel);
      // public static DashboardNumber armMinVel = 
      //    new DashboardNumber("armExtend/armMinVel", ArmConstants.armExtendMinVel);
      // public static DashboardNumber armMaxAcc = 
      //    new DashboardNumber("armExtend/armMaxAcc", ArmConstants.armExtendMaxAcc);
      // public static DashboardNumber armAllowedErr = 
      //    new DashboardNumber("armExtend/armAllowedErr", ArmConstants.armExtendAllowedErr);
      // public static DashboardNumber armGearRatio = 
      //    new DashboardNumber("armExtend/armExtendGearRatio", ArmConstants.armExtendGearRatio);
      // public static DashboardNumber armMetersPerRotation = 
      //    new DashboardNumber("armExtend/armExtendMetersPerRotation", ArmConstants.armExtendMetersPerRotation);
      // public static DashboardNumber armMinLength = 
      //    new DashboardNumber("armExtend/armExtendMinLength", ArmConstants.armExtendMinLength);
      // public static DashboardNumber armMaxLength = 
      //    new DashboardNumber("armExtend/armExtendMaxLength", ArmConstants.armExtendMaxLength);
  }
}
