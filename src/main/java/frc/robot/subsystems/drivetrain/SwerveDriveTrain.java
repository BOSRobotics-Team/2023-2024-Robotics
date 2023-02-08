package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.*;

import frc.lib.gyro.GyroIO;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.RobotOdometry;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class SwerveDriveTrain extends SubsystemBase {
    private enum DriveMode {
        NORMAL,
        X,
        CHARACTERIZATION
    };

    private GyroIO gyro;

    private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR
    private SwerveDriveOdometry swerveOdometry;
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
        new Translation2d(RobotPreferences.wheelBase.get() / 2.0, RobotPreferences.trackWidth.get() / 2.0),
        new Translation2d(RobotPreferences.wheelBase.get() / 2.0, -RobotPreferences.trackWidth.get() / 2.0),
        new Translation2d(-RobotPreferences.wheelBase.get() / 2.0, RobotPreferences.trackWidth.get() / 2.0),
        new Translation2d(-RobotPreferences.wheelBase.get() / 2.0, -RobotPreferences.trackWidth.get() / 2.0));

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        RobotPreferences.Auto.maxSpeedMetersPerSecond.get(),
        RobotPreferences.Auto.maxAccelerationMetersPerSecondSquared.get())
            .setKinematics(swerveKinematics);

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            RobotPreferences.Auto.maxAngularSpeedRadiansPerSecond.get(), 
            RobotPreferences.Auto.maxAngularSpeedRadiansPerSecondSquared.get());

    private final PIDController autoXController =
        new PIDController(RobotPreferences.Auto.pXController.get(), 
            RobotPreferences.Auto.iXController.get(), 
            RobotPreferences.Auto.dXController.get());
    private final PIDController autoYController =
        new PIDController(RobotPreferences.Auto.pYController.get(), 
            RobotPreferences.Auto.iYController.get(), 
            RobotPreferences.Auto.dYController.get());
    private final PIDController autoThetaController =
        new PIDController(RobotPreferences.Auto.pThetaController.get(), 
            RobotPreferences.Auto.iThetaController.get(), 
            RobotPreferences.Auto.dThetaController.get());
    private final ProfiledPIDController autoProfiledThetaController =
        new ProfiledPIDController(RobotPreferences.Auto.pThetaController.get(), 
            RobotPreferences.Auto.iThetaController.get(), 
            RobotPreferences.Auto.dThetaController.get(),
            SwerveDriveTrain.kThetaControllerConstraints);
          
    public SwerveDriveTrain(GyroIO gyro, SwerveModule mod0, SwerveModule mod1, SwerveModule mod2, SwerveModule mod3) {
        this.gyro = gyro;
        gyro.setGyroDirection(GYRO_DIRECTION);
        zeroGyro();
        
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.autoProfiledThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
        this.estimatedPoseWithoutGyro = new Pose2d();
        this.isFieldRelative = true;
        this.brakeMode = false;
        this.driveMode = DriveMode.NORMAL;
        this.characterizationVoltage = 0.0;
        this.centerGravity = new Translation2d();
        
        this.swerveModules[0] = mod0;
        this.swerveModules[1] = mod1;
        this.swerveModules[2] = mod2;
        this.swerveModules[3] = mod3;

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        this.swerveOdometry = new SwerveDriveOdometry(SwerveDriveTrain.swerveKinematics, getRotation(), getModulePositions());
        initLogging();
    }

    public void drive(double translation, double strafe, double rotation) {
        switch (driveMode) {
            case NORMAL:
                if (isFieldRelative) {
                    this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation, 
                        strafe, 
                        rotation, 
                        getRotation());
                } else {
                    this.chassisSpeeds = new ChassisSpeeds(
                        translation, 
                        strafe, 
                        rotation);
                }    
                SwerveModuleState[] states =
                    SwerveDriveTrain.swerveKinematics.toSwerveModuleStates(this.chassisSpeeds, this.centerGravity);
                setModuleStates(states, true, false);
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
        setModuleStates(swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerGravity));
    }

    public void setXStance() {
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);

        states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(RobotPreferences.trackWidth.get() / RobotPreferences.wheelBase.get()));
        states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(RobotPreferences.trackWidth.get() / RobotPreferences.wheelBase.get()));
        states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(RobotPreferences.trackWidth.get() / RobotPreferences.wheelBase.get()));
        states[3].angle = new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(RobotPreferences.trackWidth.get() / RobotPreferences.wheelBase.get()));
        setModuleStates(states, true, true);
    }
    
    /* Used by SwerveControllerCommand */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean isForceAngle) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotPreferences.Swerve.maxSpeed.get());
        for(SwerveModule mod : this.swerveModules){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop, isForceAngle);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        this.setModuleStates(desiredStates, false, false);
    }

    public Pose2d getPose() {
        // return this.swerveOdometry.getPoseMeters();
       return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        this.swerveOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
   * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param state the specified PathPlanner state to which is set the odometry
   */
    public void resetOdometry(PathPlannerState state) {
        setGyroOffset(state.holonomicRotation.getDegrees());

        estimatedPoseWithoutGyro =
            new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
        poseEstimator.resetPosition(
            this.getRotation(),
            this.getModulePositions(),
            new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : this.swerveModules){
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : this.swerveModules){
            positions[mod.getModuleNumber()] = mod.getPosition();
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
            return gyro.getRotation2d();
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
                if (Math.abs(mod.getState().speedMetersPerSecond) > RobotPreferences.Swerve.maxCoastVelocity_MPS.get()) {
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
    public PIDController getAutoThetaController() {
        return autoThetaController;
    }

    /**
     * Returns the PID controller used to control the robot's rotation during autonomous.
     *
     * @return the PID controller used to control the robot's rotation during autonomous
     */
    public ProfiledPIDController getAutoProfiledThetaController() {
        return autoProfiledThetaController;
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = volts;

        // invoke drive which will set the characterization voltage to each module
        drive(0.0, 0.0, 0.0);
    }

    /** Returns the average drive velocity in meters/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (SwerveModule swerveModule : swerveModules) {
            driveVelocityAverage += swerveModule.getState().speedMetersPerSecond;
        }
        return driveVelocityAverage / 4.0;
    }

    public void testModule(int moduleNumber, double drive, double angle) {
        for (SwerveModule swerveModule : swerveModules) {
            if (moduleNumber == swerveModule.getModuleNumber()) {
                swerveModule.setDesiredState(new SwerveModuleState(drive, Rotation2d.fromDegrees(angle)), true, true);
            }
          }
    }

    @Override
    public void periodic(){
        this.swerveOdometry.update(getRotation(), getModulePositions());  
        updateBrakeMode();
    }

    public void initLogging() {
        ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
        tabMain.addNumber("DriveTrain/Gyroscope Angle", this::getRotationDegrees);
        tabMain.addBoolean("DriveTrain/X-Stance On?", this::isXstance);
        tabMain.addBoolean("DriveTrain/Field-Relative Enabled?", this::getFieldRelative);

        if (DEBUGGING) {
          ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
          tab.add("DriveTrain", this);
          tab.addNumber("vx", this::getVelocityX);
          tab.addNumber("vy", this::getVelocityY);
          tab.addNumber("Pose Est X", () -> poseEstimator.getEstimatedPosition().getX());
          tab.addNumber("Pose Est Y", () -> poseEstimator.getEstimatedPosition().getY());
          tab.addNumber("Pose Est Rot", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
          tab.addNumber("CoG X", () -> this.centerGravity.getX());
          tab.addNumber("CoG Y", () -> this.centerGravity.getY());

          for(SwerveModule mod : this.swerveModules){
            // tab.addNumber("Mod " + mod.getModuleNumber() + " Cancoder", () -> mod.getCanCoder().getDegrees());
            tab.addNumber("Mod " + mod.getModuleNumber() + " Integrated", () -> mod.getPosition().angle.getDegrees());
            tab.addNumber("Mod " + mod.getModuleNumber() + " Velocity", () -> mod.getState().speedMetersPerSecond);    
          }
        }
    
        if (TESTING) {
          ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
          tab.add("Enable XStance", new InstantCommand(this::enableXstance));
          tab.add("Disable XStance", new InstantCommand(this::disableXstance));
        }
    }
}