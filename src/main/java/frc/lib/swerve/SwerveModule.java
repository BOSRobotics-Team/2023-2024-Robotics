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
import frc.robot.subsystems.drivetrain.DriveTrainConstants;

public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

  private int moduleNumber;
  private double lastAngle;
  private double maxVelocity;

  public SwerveModule(SwerveModuleIO io) {
    this.io = io;
    this.moduleNumber = io.getModuleNumber();
    this.maxVelocity = DriveTrainConstants.maxSpeed;

    lastAngle = getState().angle.getDegrees();

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

  // /**
  //  * Minimize the change in heading the desired swerve module state would require by potentially
  //  * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
  //  * appropriate scope for CTRE onboard control.
  //  *
  //  * @param desiredState The desired state.
  //  * @param currentAngle The current module angle.
  //  */
  // public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
  //   double targetAngle =
  //       placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
  //   double targetSpeed = desiredState.speedMetersPerSecond;
  //   double delta = targetAngle - currentAngle.getDegrees();

  //   if (Math.abs(delta) > 90) {
  //     targetSpeed = -targetSpeed;
  //     targetAngle = delta > 90 ? (targetAngle - 180) : (targetAngle + 180);
  //   }
  //   return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  // }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;

    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * Set this swerve module to the specified speed and angle.
   *
   * @param desiredState the desired state of the module
   * @param isOpenLoop if true, the drive motor will be set to the calculated fraction of the max
   *     velocity; if false, the drive motor will set to the specified velocity using a closed-loop
   *     controller (PID).
   * @param forceAngle if true, the module will be forced to rotate to the specified angle; if
   *     false, the module will not rotate if the velocity is less than 1% of the max velocity.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {

    // this optimization is specific to CTRE hardware; perhaps this responsibility should be demoted
    // to the hardware-specific classes.
    // desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    double currentAngleDegrees = getState().angle.getDegrees();
    double targetAngle = placeInAppropriate0To360Scope(currentAngleDegrees, desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;

    double delta = targetAngle - currentAngleDegrees;
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle - 180) : (targetAngle + 180);
    }
    desiredState = new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));

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

  private void setAngle(SwerveModuleState desiredState, boolean forceAngle) {
    // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
    // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
    // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
    // during pauses (e.g., multi-segmented auto paths).
    double angle;
    if (!forceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (maxVelocity * 0.01)) {
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
    lastAngle = 0.0;
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
  public boolean isAbsoluteEncoderConnected() {
    return io.isAbsoluteEncoderConnected();
  }
}
