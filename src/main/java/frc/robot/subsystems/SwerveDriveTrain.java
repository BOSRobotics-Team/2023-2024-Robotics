package frc.robot.subsystems;

import frc.lib.util.DriveGyro;
import frc.lib.util.RobotOdometry;
import frc.lib.util.SwerveModule;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.RobotPreferences;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
    private static final boolean DEBUGGING = false;
    private static final boolean TESTING = false;
    private enum DriveMode {
        NORMAL,
        X,
        CHARACTERIZATION
    };

    private DriveGyro gyro;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] swerveModules;
    private ChassisSpeeds chassisSpeeds;
    private Translation2d centerGravity;

    private boolean isFieldRelative;
    private boolean brakeMode;
  
    private DriveMode driveMode;
    private double characterizationVoltage;
  
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d estimatedPoseWithoutGyro;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(RobotPreferences.wheelBase() / 2.0, RobotPreferences.trackWidth() / 2.0),
        new Translation2d(RobotPreferences.wheelBase() / 2.0, -RobotPreferences.trackWidth() / 2.0),
        new Translation2d(-RobotPreferences.wheelBase() / 2.0, RobotPreferences.trackWidth() / 2.0),
        new Translation2d(-RobotPreferences.wheelBase() / 2.0, -RobotPreferences.trackWidth() / 2.0));

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        RobotPreferences.AutoConstants.maxSpeedMetersPerSecond(),
        RobotPreferences.AutoConstants.maxAccelerationMetersPerSecondSquared())
            .setKinematics(swerveKinematics);

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            RobotPreferences.AutoConstants.maxAngularSpeedRadiansPerSecond(), 
            RobotPreferences.AutoConstants.maxAngularSpeedRadiansPerSecondSquared());

    private final PIDController autoXController =
        new PIDController(RobotPreferences.AutoConstants.pXController(), 
            RobotPreferences.AutoConstants.iXController(), 
            RobotPreferences.AutoConstants.dXController());
    private final PIDController autoYController =
        new PIDController(RobotPreferences.AutoConstants.pYController(), 
            RobotPreferences.AutoConstants.iYController(), 
            RobotPreferences.AutoConstants.dYController());
    private final ProfiledPIDController autoThetaController =
        new ProfiledPIDController(RobotPreferences.AutoConstants.pThetaController(), 
            RobotPreferences.AutoConstants.iThetaController(), 
            RobotPreferences.AutoConstants.dThetaController(),
            SwerveDriveTrain.kThetaControllerConstraints);
      
    public SwerveDriveTrain(DriveGyro gyro) {
        this.gyro = gyro;
        gyro.setGyroDirection(Constants.GYRO_DIRECTION);
        zeroGyro();
        
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
        this.estimatedPoseWithoutGyro = new Pose2d();
        this.isFieldRelative = true;
        this.brakeMode = false;
        this.driveMode = DriveMode.NORMAL;
        this.characterizationVoltage = 0.0;
        this.centerGravity = new Translation2d();
        
        this.swerveModules = new SwerveModule[] {
            new SwerveModule(Constants.FRONT_LEFT_MODULE, new SwerveModuleConstants(Constants.SWERVE_CAN_BUS,
                                                                        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID, 
                                                                        Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID, 
                                                                        Constants.FRONT_LEFT_MODULE_ANGLE_ENCODER_ID, 
                                                                        RobotPreferences.frontLeftModule_AngleOffset())),
            new SwerveModule(Constants.FRONT_RIGHT_MODULE, new SwerveModuleConstants(Constants.SWERVE_CAN_BUS,
                                                                        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID, 
                                                                        Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID, 
                                                                        Constants.FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID, 
                                                                        RobotPreferences.frontRightModule_AngleOffset())),
            new SwerveModule(Constants.BACK_LEFT_MODULE, new SwerveModuleConstants(Constants.SWERVE_CAN_BUS,
                                                                        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID, 
                                                                        Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID, 
                                                                        Constants.BACK_LEFT_MODULE_ANGLE_ENCODER_ID, 
                                                                        RobotPreferences.backLeftModule_AngleOffset())),
            new SwerveModule(Constants.BACK_RIGHT_MODULE, new SwerveModuleConstants(Constants.SWERVE_CAN_BUS,
                                                                        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID, 
                                                                        Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID, 
                                                                        Constants.BACK_RIGHT_MODULE_ANGLE_ENCODER_ID, 
                                                                        RobotPreferences.backRightModule_AngleOffset())),
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        this.swerveOdometry = new SwerveDriveOdometry(SwerveDriveTrain.swerveKinematics, getRotation(), getModulePositions());

        ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
        tabMain.addNumber("Gyroscope Angle", this::getRotationDegrees);
        tabMain.addBoolean("X-Stance On?", this::isXstance);
        tabMain.addBoolean("Field-Relative Enabled?", this::getFieldRelative);
    
        if (DEBUGGING) {
          ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
          tab.add("DriveTrain", this);
          tab.addNumber("vx", this::getVelocityX);
          tab.addNumber("vy", this::getVelocityY);
          tab.addNumber("Pose Est X", () -> poseEstimator.getEstimatedPosition().getX());
          tab.addNumber("Pose Est Y", () -> poseEstimator.getEstimatedPosition().getY());
          tab.addNumber(
              "Pose Est Rot", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
          tab.addNumber("CoG X", () -> this.centerGravity.getX());
          tab.addNumber("CoG Y", () -> this.centerGravity.getY());
        }
    
        if (TESTING) {
          ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
          tab.add("Enable XStance", new InstantCommand(this::enableXstance));
          tab.add("Disable XStance", new InstantCommand(this::disableXstance));
        }
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        switch (driveMode) {
            case NORMAL:
                if (isFieldRelative) {
                    this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getRotation());
                } else {
                    this.chassisSpeeds = new ChassisSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation);
                }    
                SwerveModuleState[] states =
                    SwerveDriveTrain.swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
                setModuleStates(states, isOpenLoop);
                break;
            case CHARACTERIZATION:
                // In characterization mode, drive at the specified voltage (and turn to zero degrees)
                for (SwerveModule swerveModule : swerveModules) {
                  swerveModule.setVoltageForCharacterization(characterizationVoltage);
                }
                break;
            case X:
                this.setXStance();
                break;
        }
    }    

    /**
     * Stops the motion of the robot. Since the motors are in break mode, the robot will stop soon
     * after this method is invoked.
     */
    public void stop() {
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
        setModuleStates(states, true);
    }

    public void setXStance() {
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);

        states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(RobotPreferences.trackWidth() / RobotPreferences.wheelBase()));
        states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(RobotPreferences.trackWidth() / RobotPreferences.wheelBase()));
        states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(RobotPreferences.trackWidth() / RobotPreferences.wheelBase()));
        states[3].angle = new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(RobotPreferences.trackWidth() / RobotPreferences.wheelBase()));
        setModuleStates(states, true);
    }
    
    /* Used by SwerveControllerCommand */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotPreferences.Swerve.maxSpeed());
        for(SwerveModule mod : this.swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }    

    public Pose2d getPose() {
        return this.swerveOdometry.getPoseMeters();
//        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        this.swerveOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : this.swerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : this.swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Sets the robot's center of gravity about which it will rotate. The origin is at the center of
     * the robot. The positive x direction is forward; the positive y direction, left.
     *
     * @param x the x coordinate of the robot's center of gravity (in meters)
     * @param y the y coordinate of the robot's center of gravity (in meters)
     */
    public void setCenterGrav(double x, double y) {
        this.centerGravity = new Translation2d(x, y);
    }

    /** Resets the robot's center of gravity about which it will rotate to the center of the robot. */
    public void resetCenterGrav() {
        setCenterGrav(0.0, 0.0);
    }

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getRotation() {
        if (gyro.isConnected()) {
            return gyro.getHeading();
        } else {
            return estimatedPoseWithoutGyro.getRotation();
        }
    }

    public double getRotationDegrees() {
        return this.getRotation().getDegrees();
    }
    /**
    * Sets the rotation of the robot to the specified value. This method should only be invoked when
    * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
    * facing away from the driver station; CCW is positive.
    *
    * @param expectedYaw the rotation of the robot (in degrees)
    */
    public void setGyroOffset(double expectedYaw) {
    // There is a delay between setting the yaw on the Gyro and that change
    //      taking effect. As a result, it is recommended to never set the yaw and
    //      adjust the local offset instead.
        gyro.setHeadingOffset(expectedYaw);
        if (!gyro.isConnected()) {
            this.estimatedPoseWithoutGyro =
                new Pose2d(
                    estimatedPoseWithoutGyro.getX(),
                    estimatedPoseWithoutGyro.getY(),
                    Rotation2d.fromDegrees(expectedYaw));
        }
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : this.swerveModules){
            mod.resetToAbsolute();
        }
    }

    /**
     * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
     * stopped moving, and brake mode is enabled, disable it.
     */
    private void updateBrakeMode() {
        if (DriverStation.isEnabled() && !brakeMode) {
            brakeMode = true;
            setBrakeMode(true);
        } else {
            boolean stillMoving = false;
            for (SwerveModule mod : swerveModules) {
                if (Math.abs(mod.getState().speedMetersPerSecond) > RobotPreferences.Swerve.maxCoastVelocity_MPS()) {
                    stillMoving = true;
                }
            }
            if (brakeMode && !stillMoving) {
                brakeMode = false;
                setBrakeMode(false);
            }
        }
    }

    private void setBrakeMode(boolean enable) {
        for (SwerveModule mod : swerveModules) {
            mod.setAngleBrakeMode(enable);
            mod.setDriveBrakeMode(enable);
        }
    }

    /**
     * Returns true if field relative mode is enabled
     *
     * @return true if field relative mode is enabled
     */
    public boolean getFieldRelative() {
        return isFieldRelative;
    }

    /**
    * Enables field-relative mode. When enabled, the joystick inputs specify the velocity of the
    * robot in the frame of reference of the field.
    */
    public void enableFieldRelative() {
        isFieldRelative = true;
    }

    /**
    * Disables field-relative mode. When disabled, the joystick inputs specify the velocity of the
    * robot in the frame of reference of the robot.
    */
    public void disableFieldRelative() {
        isFieldRelative = false;
    }
    
    /**
     * Returns the desired velocity of the drivetrain in the x direction (units of m/s)
     *
     * @return the desired velocity of the drivetrain in the x direction (units of m/s)
     */
    public double getVelocityX() {
        return chassisSpeeds.vxMetersPerSecond;
    }

    /**
     * Returns the desired velocity of the drivetrain in the y direction (units of m/s)
     *
     * @return the desired velocity of the drivetrain in the y direction (units of m/s)
     */
    public double getVelocityY() {
        return chassisSpeeds.vyMetersPerSecond;
    }

    /**
    * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
    * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
    * useful when shooting. The robot cannot be driven until x-stance is disabled.
    */
    public void enableXstance() {
        this.driveMode = DriveMode.X;
        this.setXStance();
    }

    /** Disables the x-stance, allowing the robot to be driven. */
    public void disableXstance() {
        this.driveMode = DriveMode.NORMAL;
    }
   
    /**
     * Returns true if the robot is in the x-stance orientation.
     *
     * @return true if the robot is in the x-stance orientation
     */
    public boolean isXstance() {
    return this.driveMode == DriveMode.X;
    }

    /**
     * Returns the PID controller used to control the robot's x position during autonomous.
     *
     * @return the PID controller used to control the robot's x position during autonomous
     */
    public PIDController getAutoXController() {
        return autoXController;
    }

    /**
     * Returns the PID controller used to control the robot's y position during autonomous.
     *
     * @return the PID controller used to control the robot's y position during autonomous
     */
    public PIDController getAutoYController() {
        return autoYController;
    }

    /**
     * Returns the PID controller used to control the robot's rotation during autonomous.
     *
     * @return the PID controller used to control the robot's rotation during autonomous
     */
    public ProfiledPIDController getAutoThetaController() {
        return autoThetaController;
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = volts;

        // invoke drive which will set the characterization voltage to each module
        drive(new Translation2d(), 0, true);
    }

    /** Returns the average drive velocity in meters/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (SwerveModule swerveModule : swerveModules) {
            driveVelocityAverage += swerveModule.getState().speedMetersPerSecond;
        }
        return driveVelocityAverage / 4.0;
    }

    @Override
    public void periodic(){
        this.swerveOdometry.update(getRotation(), getModulePositions());  
        updateBrakeMode();
    }

    public void logPeriodic() {
        gyro.logPeriodic();

        for(SwerveModule mod : this.swerveModules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}