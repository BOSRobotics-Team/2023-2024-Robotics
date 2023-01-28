package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        driveTrain.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * scaleFactorSup.getAsDouble()), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true
        );
    }
}
