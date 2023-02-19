package frc.lib.swerve;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.swerve.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.RobotPreferences;

public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

  private int moduleNumber;
  private double lastAngle;
  private double maxVelocity;

  public SwerveModule(SwerveModuleIO io) {
    this.io = io;
    this.moduleNumber = io.getModuleNumber();
    this.lastAngle = getState().angle.getDegrees();
    this.maxVelocity = RobotPreferences.Swerve.maxSpeed.get();

    this.initLogging();
  }

  private void initLogging() {
    /* set DEBUGGING to true to view values in Shuffleboard. This is useful when determining the steer offset constants. */
    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
      ShuffleboardLayout layout =
          tab.getLayout("Mod " + moduleNumber, BuiltInLayouts.kList)
              .withPosition(moduleNumber * 3, 0)
              .withSize(3, 3);
      layout.addNumber("Mod " + moduleNumber + ": Cancoder", () -> inputs.angleAbsolutePositionDeg);
      layout.addNumber("Mod " + moduleNumber + ": Integrated", () -> inputs.anglePositionDeg);
      layout.addNumber(
          "Mod " + moduleNumber + ": Velocity", () -> inputs.driveVelocityMetersPerSec);
    }
  }

  public void setDesiredState(
      SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {
    // this optimization is specific to CTRE hardware; perhaps this responsibility should be demoted
    // to the hardware-specific classes.
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    setSpeed(desiredState, isOpenLoop);
    setAngle(desiredState, forceAngle);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / maxVelocity;
      io.setDriveMotorPercentage(percentOutput);
    } else {
      io.setDriveVelocity(desiredState.speedMetersPerSecond);
    }
  }

  private void setAngle(SwerveModuleState desiredState, boolean isForceAngle) {
    // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
    // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
    // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
    // during pauses (e.g., multi-segmented auto paths).
    double angle;
    if (!isForceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (maxVelocity * 0.01)) {
      angle = lastAngle;
    } else {
      angle = desiredState.angle.getDegrees();
    }

    io.setAnglePosition(angle);
    lastAngle = angle;
  }

  /**
   * Set the drive motor to the specified voltage. This is only used for characterization via the
   * FeedForwardCharacterization command. The module will be set to 0 degrees throughout the
   * characterization; as a result, the wheels don't need to be clamped to hold them straight.
   *
   * @param voltage the specified voltage for the drive motor
   */
  public void setVoltageForCharacterization(double voltage) {
    io.setAnglePosition(0.0);
    this.lastAngle = 0.0;
    io.setDriveMotorPercentage(voltage / 12.0);
  }

  /**
   * Get the current state of this swerve module.
   *
   * @return the current state of this swerve module
   */
  public SwerveModuleState getState() {
    double velocity = inputs.driveVelocityMetersPerSec;
    Rotation2d angle = Rotation2d.fromDegrees(inputs.anglePositionDeg);
    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Get the current position of this swerve module.
   *
   * @return the current position of this swerve module
   */
  public SwerveModulePosition getPosition() {
    double distance = inputs.driveDistanceMeters;
    Rotation2d angle = Rotation2d.fromDegrees(inputs.anglePositionDeg);
    return new SwerveModulePosition(distance, angle);
  }

  /**
   * Get the number of this swerve module.
   *
   * @return the number of this swerve module
   */
  public int getModuleNumber() {
    return moduleNumber;
  }

  /**
   * Update this swerve module's inputs and log them.
   *
   * <p>This method must be invoked by the drivetrain subsystem's periodic method.
   */
  public void updateAndProcessInputs() {
    io.updateInputs(inputs);
    // Logger.getInstance().processInputs("Mod" + moduleNumber, inputs);
  }

  /**
   * Set the brake mode of the drive motor.
   *
   * @param enable if true, the drive motor will be set to brake mode; if false, coast mode.
   */
  public void setDriveBrakeMode(boolean enable) {
    io.setDriveBrakeMode(enable);
  }

  /**
   * Set the brake mode of the angle motor.
   *
   * @param enable if true, the angle motor will be set to brake mode; if false, coast mode.
   */
  public void setAngleBrakeMode(boolean enable) {
    io.setAngleBrakeMode(enable);
  }

  /** Return if the drive motor is connected. */
  public boolean isDriveMotorConnected() {
    return io.isDriveMotorConnected();
  }

  /** Return if the angle motor is connected. */
  public boolean isAngleMotorConnected() {
    return io.isAngleMotorConnected();
  }

  /** Return if the CANCoder is connected. */
  public boolean isAngleEncoderConnected() {
    return io.isAngleEncoderConnected();
  }

  public void resetToAbsolute() {
    //        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() -
    // angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
    //        mAngleMotor.setSelectedSensorPosition(absolutePosition);
  }
}
