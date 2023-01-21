package frc.robot.subsystems;

import frc.lib.util.DriveGyro;
import frc.lib.util.SwerveModule;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public DriveGyro gyro;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.wheelBase / 2.0, Constants.trackWidth / 2.0),
        new Translation2d(Constants.wheelBase / 2.0, -Constants.trackWidth / 2.0),
        new Translation2d(-Constants.wheelBase / 2.0, Constants.trackWidth / 2.0),
        new Translation2d(-Constants.wheelBase / 2.0, -Constants.trackWidth / 2.0));

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(swerveKinematics);

        /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    public SwerveDriveTrain() {
        gyro = new DriveGyro(Constants.GYRO_ID, Constants.GYRO_CAN_BUS);
        gyro.setGyroDirection(Constants.GYRO_DIRECTION);
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, new SwerveModuleConstants(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
                                                                        Constants.FRONT_LEFT_MODULE_STEER_MOTOR, 
                                                                        Constants.FRONT_LEFT_MODULE_STEER_ENCODER, 
                                                                        Constants.SWERVE_CAN_BUS,
                                                                        Constants.FRONT_LEFT_MODULE_STEER_OFFSET)),
            new SwerveModule(1, new SwerveModuleConstants(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
                                                                        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, 
                                                                        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, 
                                                                        Constants.SWERVE_CAN_BUS,
                                                                        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)),
            new SwerveModule(2, new SwerveModuleConstants(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, 
                                                                        Constants.BACK_LEFT_MODULE_STEER_MOTOR, 
                                                                        Constants.BACK_LEFT_MODULE_STEER_ENCODER, 
                                                                        Constants.SWERVE_CAN_BUS,
                                                                        Constants.BACK_LEFT_MODULE_STEER_OFFSET)),
            new SwerveModule(3, new SwerveModuleConstants(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
                                                                        Constants.BACK_RIGHT_MODULE_STEER_MOTOR, 
                                                                        Constants.BACK_RIGHT_MODULE_STEER_ENCODER, 
                                                                        Constants.SWERVE_CAN_BUS,
                                                                        Constants.BACK_RIGHT_MODULE_STEER_OFFSET)),
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveDriveTrain.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveDriveTrain.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return gyro.getHeading();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    
    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    public void logPeriodic() {
        gyro.logPeriodic();

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}