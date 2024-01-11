package frc.lib.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;

public class SwerveModuleIONeo implements SwerveModuleIO {

    private int moduleNumber;
    private CANSparkMax mDriveMotor;
    private CANSparkMax mAngleMotor;
    private CANcoder absoluteEncoder;

    private SimpleMotorFeedforward feedForward;
    private double angleOffsetDeg;

    private RelativeEncoder angleEncoder;
    private SparkPIDController anglePID;
    
    private RelativeEncoder driveEncoder;
    private SparkPIDController drivePID;

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

    public SwerveModuleIONeo(
        int moduleNumber,
        String canBusID,
        int driveMotorID,
        int angleMotorID,
        int canCoderID,
        double angleOffsetDeg)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffsetDeg = angleOffsetDeg;
        this.feedForward =
            new SimpleMotorFeedforward(
                DriveTrainConstants.driveKS, DriveTrainConstants.driveKV, DriveTrainConstants.driveKA);

        configAbsoluteEncoder(canCoderID, canBusID);
        configAngleMotor(angleMotorID, canBusID);
        configDriveMotor(driveMotorID, canBusID);
    }

    public SwerveModuleIONeo(SwerveModuleID modID) {
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
        config.MagnetSensor.SensorDirection = DriveTrainConstants.canCoderInvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);
    }

    private void configAngleMotor(int angleMotorID, String canBusID) {
        mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = mAngleMotor.getEncoder();
        anglePID = mAngleMotor.getPIDController();

        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.enableVoltageCompensation(12);
        mAngleMotor.setSmartCurrentLimit(DriveTrainConstants.angleContinuousCurrentLimit);
        mAngleMotor.setSecondaryCurrentLimit(DriveTrainConstants.anglePeakCurrentLimit);
        mAngleMotor.setInverted(DriveTrainConstants.angleMotorInvert);
        mAngleMotor.setIdleMode(DriveTrainConstants.angleNeutralMode == NeutralMode.Coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        
        // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
        // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        angleEncoder.setPositionConversionFactor((1 / DriveTrainConstants.angleGearRatio) * 360.0); 
        angleEncoder.setPosition(getAbsoluteAngle().getDegrees() - angleOffsetDeg);

        anglePID.setFeedbackDevice(angleEncoder);
        anglePID.setP(DriveTrainConstants.angleKP);
        anglePID.setI(DriveTrainConstants.angleKI);
        anglePID.setD(DriveTrainConstants.angleKD);
        anglePID.setFF(DriveTrainConstants.angleKF);
    
        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMinInput(-1);
        anglePID.setPositionPIDWrappingMaxInput(1);

        // mAngleMotor.burnFlash();
        // TODO: Adjust this later after we know the pid loop is not crazy
        // anglePID.setOutputRange(-.25, .25);
    }

    private void configDriveMotor(int driveMotorID, String canBusID) {
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        drivePID = mDriveMotor.getPIDController();

        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.enableVoltageCompensation(12);
        mDriveMotor.setSmartCurrentLimit(DriveTrainConstants.drivePeakCurrentLimit);
        mDriveMotor.setSecondaryCurrentLimit(DriveTrainConstants.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(DriveTrainConstants.driveMotorInvert);
        mDriveMotor.setIdleMode(DriveTrainConstants.driveNeutralMode == NeutralMode.Coast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);

        mDriveMotor.setOpenLoopRampRate(DriveTrainConstants.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(DriveTrainConstants.closedLoopRamp);

        // 1/gear ratio because the wheel spins slower than the motor.
        // Multiply by the circumference to get meters per minute, divide by 60 to get meters per second.
        driveEncoder.setVelocityConversionFactor((1 / DriveTrainConstants.driveGearRatio) * DriveTrainConstants.wheelCircumference / 60.0); 
        driveEncoder.setPosition(0);

        drivePID.setFeedbackDevice(driveEncoder);
        drivePID.setP(DriveTrainConstants.driveKP);
        drivePID.setI(DriveTrainConstants.driveKI);
        drivePID.setD(DriveTrainConstants.driveKD);
        drivePID.setFF(DriveTrainConstants.driveKF);
 
        // mDriveMotor.burnFlash();
        // TODO: Remove after we know the pid loop isn't wild
        // drivePID.setOutputRange(-.5, .5);
    }

    /**
     * Get the rotation of the wheel (mechanism) as a {@link Rotation2d}
     * @return the {@link Rotation2d} describing the current rotation of the wheel
     */
    private Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue());
    }
    
    /**
     * Get the drive motor position (rotor)
     * @return the rotor position, in rots (rotations)
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition() / DriveTrainConstants.driveGearRatio;
    }

    /**
     * Get the drive motor velocity (rotor)
     * @return the rotor velocity, in rots/sec (rps)
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity() / 60 / DriveTrainConstants.driveGearRatio;
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
    
    @Override
    public void updateInputs(final SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.driveDistanceMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = getDriveVelocity();
        inputs.driveAppliedPercentage = mDriveMotor.getAppliedOutput();

        inputs.angleAbsolutePositionDeg = getAbsoluteAngle().getDegrees();
        inputs.anglePositionDeg = angleEncoder.getPosition();
        inputs.angleVelocityRevPerMin = angleEncoder.getVelocity();
        inputs.angleAppliedPercentage = mAngleMotor.getAppliedOutput();
    }

    /** Run the drive motor at the specified percentage of full power. */
    @Override
    public void setDriveMotorPercentage(double percentage) {
        mDriveMotor.set(percentage);
    }

    /** Run the drive motor at the specified velocity. */
    @Override
    public void setDriveVelocity(double velocity) {
        drivePID.setReference(
                        velocity,
                        ControlType.kVelocity,
                        0,
                        calculateFeedforward(velocity),
                        SparkPIDController.ArbFFUnits.kVoltage);
    }

    /** Run the turn motor to the specified angle. */
    @Override
    public void setAnglePosition(double degrees) {
        anglePID.setReference(degrees, ControlType.kPosition);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        mDriveMotor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setAngleBrakeMode(boolean enable) {
        // always leave the angle motor in coast mode
        mAngleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
}
