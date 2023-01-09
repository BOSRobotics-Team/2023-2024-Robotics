package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private double scaleFactor;
    private boolean updateScale = false;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.scaleFactor = 0.5;
    }

    @Override
    public void execute() {
        double yAxis = controller.getRawAxis(translationAxis);
        double xAxis = controller.getRawAxis(strafeAxis);
        double rAxis = controller.getRawAxis(rotationAxis);
        
        if (!updateScale && controller.getRawButtonPressed(5)) {
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

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis * scaleFactor;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis * scaleFactor;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis * scaleFactor * 0.5;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
