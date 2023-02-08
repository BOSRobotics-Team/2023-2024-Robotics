package frc.lib.swerve;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.swerve.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.RobotPreferences;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    private double lastAngle;

    public SwerveModule(SwerveModuleIO io){
        this.io = io;
        this.lastAngle = getState().angle.getDegrees();

        /* set DEBUGGING to true to view values in Shuffleboard. This is useful when determining the steer offset constants. */
        if (DEBUGGING) {
            ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
            tab.addNumber("Mod " + io.getModuleNumber() + ": Cancoder", () -> inputs.angleAbsolutePositionDeg);
            tab.addNumber("Mod " + io.getModuleNumber() + ": Integrated", () -> inputs.anglePositionDeg);
            tab.addNumber("Mod " + io.getModuleNumber() + ": Velocity", () -> inputs.driveVelocityMetersPerSec);
        }
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
            double percentOutput = desiredState.speedMetersPerSecond / RobotPreferences.Swerve.maxSpeed.get();
            io.setDriveMotorPercentage(percentOutput);
        }
        else {
            io.setDriveVelocity(desiredState.speedMetersPerSecond);
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean isForceAngle){
        // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
        // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
        // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
        // during pauses (e.g., multi-segmented auto paths).
        double angle;
        if (!isForceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (RobotPreferences.Swerve.maxSpeed.get() * 0.01)) {
            angle = lastAngle;
        } else {
            angle = desiredState.angle.getDegrees();
        }
        io.setAnglePosition(angle);
        lastAngle = angle;
    }

    /**
     * Get the number of this swerve module.
     *
     * @return the number of this swerve module
     */
    public int getModuleNumber() {
        return io.getModuleNumber();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            inputs.driveVelocityMetersPerSec, 
            Rotation2d.fromDegrees(inputs.anglePositionDeg));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.driveDistanceMeters, 
            Rotation2d.fromDegrees(inputs.anglePositionDeg));
    }

    public void resetToAbsolute(){
//        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
//        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Update this swerve module's inputs and log them.
     *
     * <p>This method must be invoked by the drivetrain subsystem's periodic method.
     */
    public void updateAndProcessInputs() {
        io.updateInputs(inputs);
        // Logger.getInstance().processInputs("Mod" + io.getModuleNumber(), inputs);
    }

    public void setAngleBrakeMode(boolean enable) {
        io.setAngleBrakeMode(enable);
    }

    public void setDriveBrakeMode(boolean enable) {
        io.setDriveBrakeMode(enable);
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

}