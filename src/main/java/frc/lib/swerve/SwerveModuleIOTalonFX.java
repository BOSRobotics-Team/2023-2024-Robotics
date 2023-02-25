package frc.lib.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.lib.math.Conversions;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.drivetrain.*;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with two Falcon 500 motors
 * and a CAN coder.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {

  /* PID SLOT INDEX */
  public static final int SLOT_INDEX = 0;
  public static final int TIMEOUT_MS = 100;

  private int moduleNumber;
  private WPI_TalonFX mAngleMotor;
  private WPI_TalonFX mDriveMotor;
  private WPI_CANCoder angleEncoder;
  private SimpleMotorFeedforward feedForward;
  private double angleOffsetDeg;

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
  public SwerveModuleIOTalonFX(
      int moduleNumber,
      String canBusID,
      int driveMotorID,
      int angleMotorID,
      int canCoderID,
      double angleOffsetDeg) {

    this.moduleNumber = moduleNumber;
    this.angleOffsetDeg = angleOffsetDeg;
    this.feedForward =
        new SimpleMotorFeedforward(
            RobotPreferences.Swerve.driveKS.get(),
            RobotPreferences.Swerve.driveKV.get(),
            RobotPreferences.Swerve.driveKA.get());

    configAngleEncoder(canCoderID, canBusID);
    configAngleMotor(angleMotorID, canBusID);
    configDriveMotor(driveMotorID, canBusID);

    SendableRegistry.setName(mDriveMotor, "SwerveModule " + moduleNumber, "Drive Motor");
    SendableRegistry.setName(mAngleMotor, "SwerveModule " + moduleNumber, "Angle Motor");
    SendableRegistry.setName(angleEncoder, "SwerveModule " + moduleNumber, "Angle Encoder");
  }

  public SwerveModuleIOTalonFX(SwerveModuleID modID) {
    this(
        modID.moduleNumber,
        modID.canBusID,
        modID.driveMotorID,
        modID.angleMotorID,
        modID.canCoderID,
        modID.angleOffsetDeg);
  }

  private void configAngleEncoder(int canCoderID, String canBusID) {
    angleEncoder = new WPI_CANCoder(canCoderID, canBusID);

    angleEncoder.configFactoryDefault();

    /* Swerve CANCoder Configuration */
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = DriveTrainConstants.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    angleEncoder.configAllSettings(config);
  }

  private void configAngleMotor(int angleMotorID, String canBusID) {
    mAngleMotor = new WPI_TalonFX(angleMotorID, canBusID);

    /* Swerve Angle Motor Configurations */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            DriveTrainConstants.angleEnableCurrentLimit,
            RobotPreferences.Swerve.angleContinuousCurrentLimit.get(),
            RobotPreferences.Swerve.anglePeakCurrentLimit.get(),
            RobotPreferences.Swerve.anglePeakCurrentDuration.get());
    ;
    config.slot0.kP = RobotPreferences.Swerve.angleKP.get();
    config.slot0.kI = RobotPreferences.Swerve.angleKI.get();
    config.slot0.kD = RobotPreferences.Swerve.angleKD.get();
    config.slot0.kF = RobotPreferences.Swerve.angleKF.get();

    mAngleMotor.configFactoryDefault();
    mAngleMotor.configAllSettings(config, TIMEOUT_MS);

    mAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mAngleMotor.changeMotionControlFramePeriod(101);
    mAngleMotor.clearMotionProfileHasUnderrun(TIMEOUT_MS);
    mAngleMotor.clearMotionProfileTrajectories();
    mAngleMotor.clearStickyFaults(TIMEOUT_MS);
    mAngleMotor.selectProfileSlot(0, 0);

    mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 9, TIMEOUT_MS);
    mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19, TIMEOUT_MS);
    mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 100, TIMEOUT_MS);
    mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 101, TIMEOUT_MS);
    mAngleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 102, TIMEOUT_MS);

    mAngleMotor.setInverted(DriveTrainConstants.angleMotorInvert);
    mAngleMotor.setNeutralMode(DriveTrainConstants.angleNeutralMode);

    double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffsetDeg, DriveTrainConstants.angleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
    mAngleMotor.configVoltageCompSaturation(12); // default 12v voltage compensation for motors
    mAngleMotor.enableVoltageCompensation(true);
  }

  private void configDriveMotor(int driveMotorID, String canBusID) {
    mDriveMotor = new WPI_TalonFX(driveMotorID, canBusID);

    /* Swerve Drive Motor Configuration */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            DriveTrainConstants.driveEnableCurrentLimit,
            RobotPreferences.Swerve.driveContinuousCurrentLimit.get(),
            RobotPreferences.Swerve.drivePeakCurrentLimit.get(),
            RobotPreferences.Swerve.drivePeakCurrentDuration.get());
    ;
    config.slot0.kP = RobotPreferences.Swerve.driveKP.get();
    config.slot0.kI = RobotPreferences.Swerve.driveKI.get();
    config.slot0.kD = RobotPreferences.Swerve.driveKD.get();
    config.slot0.kF = RobotPreferences.Swerve.driveKF.get();
    config.openloopRamp = RobotPreferences.Swerve.openLoopRamp.get();
    config.closedloopRamp = RobotPreferences.Swerve.closedLoopRamp.get();

    mDriveMotor.configFactoryDefault();
    mDriveMotor.configAllSettings(config, TIMEOUT_MS);

    mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mDriveMotor.changeMotionControlFramePeriod(101);
    mDriveMotor.clearMotionProfileHasUnderrun(TIMEOUT_MS);
    mDriveMotor.clearMotionProfileTrajectories();
    mDriveMotor.clearStickyFaults(TIMEOUT_MS);
    mDriveMotor.selectProfileSlot(0, 0);

    mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 9, TIMEOUT_MS);
    mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 19, TIMEOUT_MS);
    mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 100, TIMEOUT_MS);
    mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 101, TIMEOUT_MS);
    mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 102, TIMEOUT_MS);

    mDriveMotor.setInverted(DriveTrainConstants.driveMotorInvert);
    mDriveMotor.setNeutralMode(DriveTrainConstants.driveNeutralMode);

    mDriveMotor.configVoltageCompSaturation(12); // default 12v voltage compensation for motors
    mDriveMotor.enableVoltageCompensation(true);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  private Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
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
    inputs.drivePositionDeg =
        Conversions.falconToDegrees(
            mDriveMotor.getSelectedSensorPosition(), DriveTrainConstants.driveGearRatio);
    inputs.driveDistanceMeters =
        Conversions.falconToMeters(
            mDriveMotor.getSelectedSensorPosition(),
            DriveTrainConstants.wheelCircumference,
            DriveTrainConstants.driveGearRatio);
    inputs.driveVelocityMetersPerSec =
        Conversions.falconToMPS(
            mDriveMotor.getSelectedSensorVelocity(),
            DriveTrainConstants.wheelCircumference,
            DriveTrainConstants.driveGearRatio);
    inputs.driveAppliedPercentage = mDriveMotor.getMotorOutputPercent();
    inputs.driveCurrentAmps = new double[] {mDriveMotor.getStatorCurrent()};
    inputs.driveTempCelsius = new double[] {mDriveMotor.getTemperature()};

    inputs.angleAbsolutePositionDeg = angleEncoder.getAbsolutePosition();
    inputs.anglePositionDeg =
        Conversions.falconToDegrees(
            mAngleMotor.getSelectedSensorPosition(), DriveTrainConstants.angleGearRatio);
    inputs.angleVelocityRevPerMin =
        Conversions.falconToRPM(
            mAngleMotor.getSelectedSensorVelocity(), DriveTrainConstants.angleGearRatio);
    inputs.angleAppliedPercentage = mAngleMotor.getMotorOutputPercent();
    inputs.angleCurrentAmps = new double[] {mAngleMotor.getStatorCurrent()};
    inputs.angleTempCelsius = new double[] {mAngleMotor.getTemperature()};

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
    mDriveMotor.set(ControlMode.PercentOutput, percentage);
  }

  /** Run the drive motor at the specified velocity. */
  @Override
  public void setDriveVelocity(double velocity) {
    double ticksPerSecond =
        Conversions.MPSToFalcon(
            velocity, DriveTrainConstants.wheelCircumference, DriveTrainConstants.driveGearRatio);
    mDriveMotor.set(
        ControlMode.Velocity,
        ticksPerSecond,
        DemandType.ArbitraryFeedForward,
        calculateFeedforward(velocity));
  }

  /** Run the turn motor to the specified angle. */
  @Override
  public void setAnglePosition(double degrees) {
    mAngleMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(degrees, DriveTrainConstants.angleGearRatio));
  }

  /** Enable or disable brake mode on the drive motor. */
  @Override
  public void setDriveBrakeMode(boolean enable) {
    mDriveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /** Enable or disable brake mode on the turn motor. */
  @Override
  public void setAngleBrakeMode(boolean enable) {
    // always leave the angle motor in coast mode
    mAngleMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public boolean isDriveMotorConnected() {
    return (mDriveMotor.getLastError() == ErrorCode.OK);
  }

  @Override
  public boolean isAngleMotorConnected() {
    return (mAngleMotor.getLastError() == ErrorCode.OK);
  }

  @Override
  public boolean isAngleEncoderConnected() {
    return (angleEncoder.getLastError() == ErrorCode.OK);
  }
}
