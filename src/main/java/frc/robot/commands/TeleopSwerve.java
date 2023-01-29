package frc.robot.commands;

import frc.robot.RobotPreferences;
import frc.robot.subsystems.SwerveDriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {
    private SwerveDriveTrain driveTrain;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier scaleFactorSup;

    public TeleopSwerve(SwerveDriveTrain driveTrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier scaleFactorSup ) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.scaleFactorSup = scaleFactorSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double deadBand = RobotPreferences.stickDeadband();
        double maxSpeed = RobotPreferences.Swerve.maxSpeed() * scaleFactorSup.getAsDouble();

        double translationVal = this.scaleController(translationSup.getAsDouble(), deadBand) * maxSpeed;
        double strafeVal = this.scaleController(strafeSup.getAsDouble(), deadBand) * maxSpeed;
        double rotationVal = this.scaleController(rotationSup.getAsDouble(),deadBand) * RobotPreferences.Swerve.maxAngularVelocity();

        /* Drive */
        driveTrain.drive(translationVal, strafeVal, rotationVal);
    }

    public double scaleController(double value, double deadBand) {
        value = MathUtil.applyDeadband(value, deadBand);
        // Square the axis - more natural for drivers
        return Math.copySign(value * value, value);
    }
}
