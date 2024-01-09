package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.gyro.GyroIO;
import frc.lib.gyro.GyroIO.GyroIOInputs;
import frc.lib.limelightvision.LimelightHelpers;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.RobotOdometry;

import frc.robot.AutoConstants;
import frc.robot.Constants;
import frc.robot.testsystem.TestableSubsytem;

public class Drivetrain extends TestableSubsytem {
  private final GyroIO gyro;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  private final PIDController autoXController =
      new PIDController(
          AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
  private final PIDController autoYController =
      new PIDController(
          AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController);
  private final PIDController autoThetaController =
      new PIDController(
          AutoConstants.kPThetaController,
          AutoConstants.kIThetaController,
          AutoConstants.kDThetaController);

  /* Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
  //  private final double trackwidthMeters = DriveTrainConstants.getTrackwidth;
  //  private final double wheelbaseMeters = DriveTrainConstants.getWheelbase;
   public final SwerveDriveKinematics kinematics = DriveTrainConstants.swerveKinematics;

  private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR

  private final Field2d m_field = new Field2d();

  private Translation2d centerGravity;

  private final SwerveModulePosition[] swerveModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveModulePosition[] prevSwerveModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveModuleState[] swerveModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private Pose2d estimatedPoseWithoutGyro = new Pose2d();

  private boolean isFieldRelative;

  private double gyroOffset;

  private ChassisSpeeds chassisSpeeds;

  private SwerveDrivePoseEstimator poseEstimator;
  private boolean brakeMode = false;

  private DriveMode driveMode = DriveMode.NORMAL;
  private double characterizationVoltage = 0.0;

  public Drivetrain (
      GyroIO gyro, SwerveModule mod0, SwerveModule mod1, SwerveModule mod2, SwerveModule mod3) {
    this.gyro = gyro;
    this.swerveModules[0] = mod0;
    this.swerveModules[1] = mod1;
    this.swerveModules[2] = mod2;
    this.swerveModules[3] = mod3;

    this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.centerGravity = new Translation2d();

    this.zeroGyroscope();

    this.isFieldRelative = true;

    this.gyroOffset = 0;

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    // getModulePositions();
    // this.swerveOdometry =
    //     new SwerveDriveOdometry(swerveKinematics, getRotation(), swerveModulePositions);

// Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController), // Translation PID constants
            new PIDConstants(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController), // Rotation PID constants
            AutoConstants.kMaxSpeedMetersPerSecond, //4.5, // Max module speed, in m/s
            DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {return false;},
        this // Reference to this subsystem to set requirements
    );

    initLogging();

  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    setGyroOffset(0.0);
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive. This method should always be invoked instead of obtaining the yaw directly from the
   * Pigeon as the local offset needs to be added. If the gyro is not connected, the rotation from
   * the estimated pose is returned.
   *
   * @return the rotation of the robot
   */
  public Rotation2d getRotation() {
    if (gyroInputs.connected) {
      return Rotation2d.fromDegrees(gyroInputs.positionDeg + this.gyroOffset);
    } else {
      DriverStation.reportWarning(
          "Getting estimated pose for rotation - Gyro "
              + (gyroInputs.positionDeg + this.gyroOffset)
              + " degrees",
          false);
      return estimatedPoseWithoutGyro.getRotation();
    }
  }

  /**
   * 
   * @return The tilt of the robot
   */
  public Rotation2d getTilt() {
    if (gyroInputs.connected) {
      return Rotation2d.fromDegrees(gyroInputs.pitchDeg);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public double getPitch() {
    if (gyroInputs.connected) {
      return gyroInputs.pitchDeg;
    } else {
      return 0.0;
    }
  }

  public double getRoll() {
    if (gyroInputs.connected) {
      return gyroInputs.rollDeg;
    } else {
      return 0.0;
    }
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

    DriverStation.reportWarning("Set expected yaw " + expectedYaw, false);

    if (gyroInputs.connected) {
      this.gyroOffset = expectedYaw - gyroInputs.positionDeg;
    } else {
      this.gyroOffset = 0;
      this.estimatedPoseWithoutGyro =
          new Pose2d(
              estimatedPoseWithoutGyro.getX(),
              estimatedPoseWithoutGyro.getY(),
              Rotation2d.fromDegrees(expectedYaw));
    }
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field to the lower left corner (i.e., the corner of the field to
   * the driver's right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getPose() {
    // return this.swerveOdometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param state the specified PathPlanner state to which is set the odometry
   */
  public void resetOdometry(State state) {
    setGyroOffset(state.targetHolonomicRotation.getDegrees());

    getModulePositions();

    estimatedPoseWithoutGyro =
        new Pose2d(state.positionMeters, state.targetHolonomicRotation);
    poseEstimator.resetPosition(
        this.getRotation(),
        swerveModulePositions,
        new Pose2d(state.positionMeters, state.targetHolonomicRotation));
  }

  public void resetOdometry(Pose2d state) {
    setGyroOffset(state.getRotation().getDegrees());

    getModulePositions();

    estimatedPoseWithoutGyro =
        new Pose2d(state.getX(), state.getY(), state.getRotation());
    poseEstimator.resetPosition(
        this.getRotation(),
        swerveModulePositions,
        new Pose2d(state.getX(), state.getY(), state.getRotation()));
  }

  public void resetPose(Pose2d pose) {
    setGyroOffset(pose.getRotation().getDegrees());

    getModulePositions();

    estimatedPoseWithoutGyro = pose;
    poseEstimator.resetPosition(this.getRotation(), swerveModulePositions, pose);
    // this.swerveOdometry.resetPosition(getRotation(), swerveModulePositions, pose);
  }

  private boolean hasLimelight() {
    return LimelightHelpers.getFiducialID(Constants.LIMELIGHTNAME) != -1;
  }

  public void resetPoseRotationToGyro() {
    getModulePositions();

    poseEstimator.resetPosition(
        this.getRotation(),
        swerveModulePositions,
        new Pose2d(this.getPose().getTranslation(), this.getRotation()));
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference of the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
   * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * <p>If the drive mode is XSTANCE, the robot will ignore the specified velocities and turn the
   * swerve modules into the x-stance orientation.
   *
   * <p>If the drive mode is CHARACTERIZATION, the robot will ignore the specified velocities and
   * run the characterization routine.
   *
   * @param translationXSupplier the desired velocity in the x direction (m/s)
   * @param translationYSupplier the desired velocity in the y direction (m/s)
   * @param rotationSupplier the desired rotational velcoity (rad/s)
   */
  public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
    switch (driveMode) {
      case NORMAL:
        if (isFieldRelative) {
          chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xVelocity, yVelocity, rotationalVelocity, getRotation());
        } else {
          chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);
        }
        SwerveModuleState[] swerveModuleStates =
            kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
        SwerveDriveKinematics.desaturateWheelSpeeds(
              swerveModuleStates, DriveTrainConstants.maxSpeed);// getRobotMaxVelocity());
  
        this.setSwerveModuleStates(swerveModuleStates, true, false);
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

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    switch (driveMode) {
      case NORMAL:
        SwerveModuleState[] swerveModuleStates =
          kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
        SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveTrainConstants.maxSpeed);

        for (SwerveModule swerveModule : swerveModules) {
          swerveModule.setDesiredState(
            swerveModuleStates[swerveModule.getModuleNumber()], true, false);
        }
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
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
    this.setSwerveModuleStates(states);
  }

  /**
   * This method is invoked each iteration of the scheduler. Typically, when using a command-based
   * model, subsystems don't override the periodic method. However, the drivetrain needs to
   * continually update the odometry of the robot, update and log the gyro and swerve module inputs,
   * update brake mode, and update the tunable values.
   */
  @Override
  public void periodic() {

    // update and log gyro inputs
    gyro.updateInputs(gyroInputs);

    // update and log the swerve moudles inputs
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.updateAndProcessInputs();
    }

    // update estimated poses
    getModulePositions();
    getModuleStates();

    // if the gyro is not connected, use the swerve module positions to estimate the robot's
    // rotation
    if (!gyroInputs.connected) {
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int index = 0; index < moduleDeltas.length; index++) {
        SwerveModulePosition current = swerveModulePositions[index];
        SwerveModulePosition previous = prevSwerveModulePositions[index];

        moduleDeltas[index] =
            new SwerveModulePosition(
                current.distanceMeters - previous.distanceMeters, current.angle);
        previous.distanceMeters = current.distanceMeters;
      }

      Twist2d twist = kinematics.toTwist2d(moduleDeltas);

      estimatedPoseWithoutGyro = estimatedPoseWithoutGyro.exp(twist);
    }

    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), this.getRotation(), swerveModulePositions);

    if (this.hasLimelight()) {
      Pose2d limelightPose2d = LimelightHelpers.getBotPose2d(Constants.LIMELIGHTNAME);
      double tl = LimelightHelpers.getLatency_Pipeline(Constants.LIMELIGHTNAME);

      poseEstimator.addVisionMeasurement(limelightPose2d, Timer.getFPGATimestamp() - tl / 1000);
      SmartDashboard.putBoolean("HasLimelight", true);
    } else {
      SmartDashboard.putBoolean("HasLimelight", false);
    }

    // update the brake mode based on the robot's velocity and state (enabled/disabled)
    updateBrakeMode();

    // // update tunables
    // if (autoDriveKp.hasChanged() || autoDriveKi.hasChanged() || autoDriveKd.hasChanged()) {
    //   autoXController.setPID(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
    //   autoYController.setPID(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
    // }
    // if (autoTurnKp.hasChanged() || autoTurnKi.hasChanged() || autoTurnKd.hasChanged()) {
    //   autoThetaController.setPID(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());
    // }
    // // log poses, 3D geometry, and swerve module states, gyro offset
    // Pose2d poseEstimatorPose = poseEstimator.getEstimatedPosition();
    // Logger.getInstance().recordOutput("Odometry/RobotNoGyro", estimatedPoseWithoutGyro);
    // Logger.getInstance().recordOutput("Odometry/Robot", poseEstimatorPose);
    // Logger.getInstance().recordOutput("3DField", new Pose3d(poseEstimatorPose));
    // Logger.getInstance().recordOutput("SwerveModuleStates", states);
    // Logger.getInstance().recordOutput(SUBSYSTEM_NAME + "/gyroOffset", this.gyroOffset);

    // this.swerveOdometry.update(getRotation(), swerveModulePositions);
    // m_field.setRobotPose(this.swerveOdometry.getPoseMeters());
    m_field.setRobotPose(poseEstimator.getEstimatedPosition());
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
        if (Math.abs(mod.getState().speedMetersPerSecond)
            > DriveTrainConstants.maxCoastVelocity_MPS) {
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
   * Sets each of the swerve modules based on the specified corresponding swerve module state.
   * Incorporates the configured feedforward when setting each swerve module. The order of the
   * states in the array must be front left, front right, back left, back right.
   *
   * <p>This method is invoked by the FollowPath autonomous command.
   *
   * @param states the specified swerve module state for each swerve module
   */
  /* Used by SwerveControllerCommand */
  public void setSwerveModuleStates(
      SwerveModuleState[] states, boolean isOpenLoop, boolean forceAngle) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.maxSpeed);

    for (SwerveModule mod : this.swerveModules) {
      mod.setDesiredState(states[mod.getModuleNumber()], isOpenLoop, forceAngle);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    this.setSwerveModuleStates(states, false, false);
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
    this.isFieldRelative = true;
  }

  /**
   * Disables field-relative mode. When disabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the robot.
   */
  public void disableFieldRelative() {
    this.isFieldRelative = false;
  }

  /**
   * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting.
   */
  public void setXStance() {
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
    states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(TRACKWIDTH / WHEELBASE));
    states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(TRACKWIDTH / WHEELBASE));
    states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(TRACKWIDTH / WHEELBASE));
    states[3].angle = new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(TRACKWIDTH / WHEELBASE));
    setSwerveModuleStates(states, true, true);
  }

  public void getModulePositions() {
    for (int i = 0; i < 4; i++) {
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }
  }

  public void getModuleStates() {
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
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

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
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
        swerveModule.setDesiredState(
            new SwerveModuleState(drive, Rotation2d.fromDegrees(angle)), true, true);
      }
    }
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public double getRotationDegrees() {
    return this.getRotation().getDegrees();
  }

  public SwerveModule getSwerveModule(int moduleNumber) {
    return swerveModules[moduleNumber];
  }

  public void initLogging() {
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    ShuffleboardLayout layout =
        tabMain.getLayout("DriveTrain", BuiltInLayouts.kList).withPosition(8, 0).withSize(4, 4);
    layout.addNumber("DriveTrain/Gyroscope Angle", this::getRotationDegrees);
    layout.addNumber("DriveTrain/Gyroscope Pitch", this::getPitch);
    layout.addNumber("DriveTrain/Gyroscope Roll", this::getRoll);
    layout.addBoolean("DriveTrain/X-Stance On?", this::isXstance);
    layout.addBoolean("DriveTrain/Field-Relative Enabled?", this::getFieldRelative);

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
      tab.add("DriveTrain", this).withPosition(0, 0).withSize(3, 1);
      tab.addNumber("vx", this::getVelocityX);
      tab.addNumber("vy", this::getVelocityY);
      tab.addNumber("Pose Est X", () -> poseEstimator.getEstimatedPosition().getX());
      tab.addNumber("Pose Est Y", () -> poseEstimator.getEstimatedPosition().getY());
      tab.addNumber(
          "Pose Est Rot", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
      tab.addNumber("CoG X", () -> this.centerGravity.getX());
      tab.addNumber("CoG Y", () -> this.centerGravity.getY());
      tab.add(m_field);
    }

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
      tab.add("Enable XStance", Commands.runOnce(this::enableXstance, this));
      tab.add("Disable XStance", Commands.runOnce(this::disableXstance, this));
    }
  }

  private enum DriveMode {
    NORMAL,
    X,
    CHARACTERIZATION
  }

  /* Tests for Drive Train Subsystem */
  @Override
  public TestStates Test(String test) {
    
    switch (test) { }
    return TestStates.NOT_IMPLEMENTED;

  }

}
