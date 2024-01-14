package frc.lib.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.lib.math.Conversions;
import frc.robot.subsystems.drivetrain.*;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with two Falcon 500 motors
 * and a CAN coder.
 */
public class SwerveModuleIOTalonFXP6 implements SwerveModuleIO {

  /* PID SLOT INDEX */
  public static final int SLOT_INDEX = 0;
  public static final int TIMEOUT_MS = 100;

  private int moduleNumber;
  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder absoluteEncoder;
  private Rotation2d angleOffset = new Rotation2d();
  private SimpleMotorFeedforward feedForward;

  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final PositionDutyCycle anglePosition = new PositionDutyCycle(0);

  /**
   * Make a new SwerveModuleIOTalonFX object.
   *
   * @param moduleNumber the module number
   * @param canBusID name of the CAN bus
   * @param driveMotorID the CAN ID of the drive motor
   * @param angleMotorID the CAN ID of the angle motor
   * @param canCoderID the CAN ID of the CANcoder
   * @param angleOffsetDeg the absolute offset of the angle encoder in degrees
   */
  public SwerveModuleIOTalonFXP6(
      int moduleNumber,
      String canBusID,
      int driveMotorID,
      int angleMotorID,
      int canCoderID,
      double angleOffsetDeg) {

    this.moduleNumber = moduleNumber;
    this.angleOffset = Rotation2d.fromDegrees(angleOffsetDeg);
    this.feedForward =
        new SimpleMotorFeedforward(
            DriveTrainConstants.driveKS, DriveTrainConstants.driveKV, DriveTrainConstants.driveKA);

    configAbsoluteEncoder(canCoderID, canBusID);
    configAngleMotor(angleMotorID, canBusID);
    configDriveMotor(driveMotorID, canBusID);

    SendableRegistry.setName(mDriveMotor, "SwerveModule " + moduleNumber, "Drive Motor");
    SendableRegistry.setName(mAngleMotor, "SwerveModule " + moduleNumber, "Angle Motor");
    SendableRegistry.setName(absoluteEncoder, "SwerveModule " + moduleNumber, "Angle Encoder");
  }

  public SwerveModuleIOTalonFXP6(SwerveModuleID modID) {
    this(
        modID.moduleNumber,
        modID.canBusID,
        modID.driveMotorID,
        modID.angleMotorID,
        modID.canCoderID,
        modID.angleOffsetDeg);
  }

  private void configAbsoluteEncoder(int canCoderID, String canBusID) {
    absoluteEncoder = new CANcoder(canCoderID, canBusID);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection =
        DriveTrainConstants.canCoderInvert
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    absoluteEncoder.getConfigurator().apply(config);
  }

  private void configAngleMotor(int angleMotorID, String canBusID) {
    mAngleMotor = new TalonFX(angleMotorID, canBusID);

    /* Swerve Angle Motor Configurations */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        DriveTrainConstants.angleMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        DriveTrainConstants.angleNeutralMode == NeutralMode.Brake
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    /* Gear Ratio Config */
    config.Feedback.SensorToMechanismRatio = DriveTrainConstants.angleGearRatio;
    config.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = DriveTrainConstants.angleEnableCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = DriveTrainConstants.angleContinuousCurrentLimit;
    config.CurrentLimits.SupplyCurrentThreshold = DriveTrainConstants.anglePeakCurrentLimit;
    config.CurrentLimits.SupplyTimeThreshold = DriveTrainConstants.anglePeakCurrentDuration;

    /* PID Config */
    config.Slot0.kP = DriveTrainConstants.angleKP;
    config.Slot0.kI = DriveTrainConstants.angleKI;
    config.Slot0.kD = DriveTrainConstants.angleKD;
    config.Slot0.kV = DriveTrainConstants.angleKF;

    mAngleMotor.getConfigurator().apply(config, TIMEOUT_MS);
    double absolutePosition = getAbsoluteAngle().getRotations() - angleOffset.getRotations();
    mAngleMotor.getConfigurator().setPosition(absolutePosition);
  }

  private void configDriveMotor(int driveMotorID, String canBusID) {
    mDriveMotor = new TalonFX(driveMotorID, canBusID);

    /* Swerve Drive Motor Configuration */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        DriveTrainConstants.driveMotorInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        DriveTrainConstants.driveNeutralMode == NeutralMode.Brake
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    /* Gear Ratio Config */
    config.Feedback.SensorToMechanismRatio = DriveTrainConstants.driveGearRatio;

    /* Current Limiting */
    config.CurrentLimits.SupplyCurrentLimitEnable = DriveTrainConstants.driveEnableCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = DriveTrainConstants.driveContinuousCurrentLimit;
    config.CurrentLimits.SupplyCurrentThreshold = DriveTrainConstants.drivePeakCurrentLimit;
    config.CurrentLimits.SupplyTimeThreshold = DriveTrainConstants.drivePeakCurrentDuration;

    /* PID Config */
    config.Slot0.kP = DriveTrainConstants.driveKP;
    config.Slot0.kI = DriveTrainConstants.driveKI;
    config.Slot0.kD = DriveTrainConstants.driveKD;
    config.Slot0.kV = DriveTrainConstants.driveKF;

    /* Open and Closed Loop Ramping */
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveTrainConstants.openLoopRamp;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DriveTrainConstants.openLoopRamp;

    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveTrainConstants.closedLoopRamp;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveTrainConstants.closedLoopRamp;
    mDriveMotor.getConfigurator().apply(config, TIMEOUT_MS);
    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  private Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue());
  }

  private double calculateFeedforward(double velocity) {
    double percentage = this.feedForward.calculate(velocity);
    // clamp the voltage to the maximum voltage
    if (percentage > 1.0) {
      return 1.0;
    }
    return percentage;
  }

  @Override
  public int getModuleNumber() {
    return this.moduleNumber;
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.driveDistanceMeters =
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValue(), DriveTrainConstants.wheelCircumference);
    inputs.driveVelocityMetersPerSec =
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValue(), DriveTrainConstants.wheelCircumference);
    inputs.driveAppliedPercentage = mDriveMotor.getDutyCycle().getValue();

    inputs.angleAbsolutePositionDeg = getAbsoluteAngle().getDegrees();
    inputs.anglePositionDeg = mAngleMotor.getPosition().getValue() * 360.0;
    inputs.angleVelocityRevPerMin = mAngleMotor.getVelocity().getValue();
    inputs.angleAppliedPercentage = mAngleMotor.getDutyCycle().getValue();

    /*  // update tunables
    if (driveKp.hasChanged()
        || driveKi.hasChanged()
        || driveKd.hasChanged()
        || turnKp.hasChanged()
        || turnKi.hasChanged()
        || turnKd.hasChanged()) {
      mDriveMotor.config_kP(SLOT_INDEX, driveKp.get());
      mDriveMotor.config_kI(SLOT_INDEX, driveKi.get());
      mDriveMotor.config_kD(SLOT_INDEX, driveKd.get());
      mAngleMotor.config_kP(SLOT_INDEX, turnKp.get());
      mAngleMotor.config_kI(SLOT_INDEX, turnKi.get());
      mAngleMotor.config_kD(SLOT_INDEX, turnKd.get());
    } */
  }

  /** Run the drive motor at the specified percentage of full power. */
  @Override
  public void setDriveMotorPercentage(double percentage) {
    driveDutyCycle.Output = percentage;
    mDriveMotor.setControl(driveDutyCycle);
  }

  /** Run the drive motor at the specified velocity. */
  @Override
  public void setDriveVelocity(double velocity) {
    driveVelocity.Slot = 0;
    driveVelocity.Velocity = Conversions.MPSToRPS(velocity, DriveTrainConstants.wheelCircumference);
    driveVelocity.FeedForward = calculateFeedforward(velocity);
    mDriveMotor.setControl(driveVelocity);
  }

  /** Run the turn motor to the specified angle. */
  @Override
  public void setAnglePosition(double degrees) {
    anglePosition.Slot = 0;
    anglePosition.Position = Rotation2d.fromDegrees(degrees).getRotations();

    mAngleMotor.setControl(anglePosition);

    // mAngleMotor.setControl(anglePosition.withPosition(Rotation2d.fromDegrees(degrees).getRotations()));
  }

  /** Enable or disable brake mode on the drive motor. */
  @Override
  public void setDriveBrakeMode(boolean enable) {
    mDriveMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** Enable or disable brake mode on the turn motor. */
  @Override
  public void setAngleBrakeMode(boolean enable) {
    // always leave the angle motor in coast mode
    mAngleMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public boolean isDriveMotorConnected() {
    return (mDriveMotor.isAlive());
  }

  @Override
  public boolean isAngleMotorConnected() {
    return (mAngleMotor.isAlive());
  }

  @Override
  public boolean isAbsoluteEncoderConnected() {
    return (absoluteEncoder.getFault_Hardware().getValue());
  }
}
