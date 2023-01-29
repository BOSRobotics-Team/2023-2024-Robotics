package frc.lib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.RobotPreferences;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private final static CTREConfigs ctreConfigs = new CTREConfigs();

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RobotPreferences.Swerve.driveKS(), 
                                                                    RobotPreferences.Swerve.driveKV(), 
                                                                    RobotPreferences.Swerve.driveKA());

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, moduleConstants.canBus);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, moduleConstants.canBus);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, moduleConstants.canBus);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle){
        // this optimization is specific to CTRE hardware; perhaps this responsibility should be demoted
        // to the hardware-specific classes.
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState, forceAngle);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / RobotPreferences.Swerve.maxSpeed();
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean isForceAngle){
        // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
        // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
        // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
        // during pauses (e.g., multi-segmented auto paths).
        if (!isForceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (RobotPreferences.Swerve.maxSpeed() * 0.01)) {
            mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(lastAngle.getDegrees(), Constants.Swerve.angleGearRatio));
        } else {
            mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredState.angle.getDegrees(), Constants.Swerve.angleGearRatio));
            lastAngle = desiredState.angle;
        }
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(SwerveModule.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(SwerveModule.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(SwerveModule.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public void setAngleBrakeMode(boolean enable) {
        mAngleMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setDriveBrakeMode(boolean enable) {
        mDriveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Set the drive motor to the specified voltage. This is only used for characterization via the
     * FeedForwardCharacterization command. The module will be set to 0 degrees throughout the
     * characterization; as a result, the wheels don't need to be clamped to hold them straight.
     *
     * @param voltage the specified voltage for the drive motor
     */
    public void setVoltageForCharacterization(double voltage) {
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(0.0, Constants.Swerve.angleGearRatio));
        lastAngle = new Rotation2d();
        mDriveMotor.set(ControlMode.PercentOutput, voltage / 12.0);
    }

}