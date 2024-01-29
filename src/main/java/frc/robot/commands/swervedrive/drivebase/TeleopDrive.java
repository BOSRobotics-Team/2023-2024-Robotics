// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;

  /**
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(
      SwerveSubsystem swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier omega,
      BooleanSupplier driveMode) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = vX.getAsDouble();
    double yVel = vY.getAsDouble();
    double rVel = omega.getAsDouble();
    boolean dMode = driveMode.getAsBoolean();

    double maxVel = swerve.getMaximumVelocity();
    double maxAngularVel = swerve.getMaximumAngularVelocity();

    double xVelocity = xVel * xVel * xVel * maxVel;
    double yVelocity = yVel * yVel * yVel * maxVel;
    double angVelocity = rVel * rVel * rVel * maxAngularVel;

    if (DEBUGGING) {
      SmartDashboard.putNumber("vX", xVelocity);
      SmartDashboard.putNumber("vY", yVelocity);
      SmartDashboard.putNumber("omega", angVelocity);
    }

    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity, yVelocity), angVelocity, dMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
