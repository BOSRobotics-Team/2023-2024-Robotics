package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
 //   private double scaleFactor;
 //   private boolean updateScale = false;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
   //     this.scaleFactor = 0.5;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /*if (!updateScale && controller.getRawButtonPressed(5)) {
            updateScale = true;
            scaleFactor -= 0.05;
            System.out.println("Setting scaleFactor to " + scaleFactor);
        }
        if (updateScale && controller.getRawButtonReleased(5)) {
            updateScale = false;
        }
        if (!updateScale && controller.getRawButtonPressed(6)) {
            updateScale = true;
            scaleFactor += 0.05;
            System.out.println("Setting scaleFactor to " + scaleFactor);
        }
        if (updateScale && controller.getRawButtonReleased(6)) {
            updateScale = false;
        }
        
        if (scaleFactor > 1.0)
            scaleFactor = 1.0;
        else if (scaleFactor < 0.05)
            scaleFactor = 0.05;
        */
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
