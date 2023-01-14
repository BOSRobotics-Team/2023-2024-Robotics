package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import javax.sql.PooledConnection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private IntSupplier povSup;
    private double scaleFactor = 0.5;
    private boolean updateScale = false;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier povSup ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.povSup = povSup;
        this.scaleFactor = 0.5;
        this.updateScale = false;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        int povVal = povSup.getAsInt();
        if (updateScale && povVal == -1) {
            updateScale = false;
        }
        if (!updateScale && povVal >= 0) {
            if (povVal == 0) {
                scaleFactor += 0.05;
                System.out.println("Setting scaleFactor to " + scaleFactor);
                updateScale = true;
            } else if (povVal == 180) {
                scaleFactor -= 0.05;
                System.out.println("Setting scaleFactor to " + scaleFactor);
                updateScale = true;
            }
        }
        scaleFactor = MathUtil.clamp(scaleFactor, 0.05, 1.0);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal * scaleFactor, strafeVal * scaleFactor).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
