// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** An example command that uses an example subsystem. */
public class VisionCommand extends Command {

  private final VisionSubsystem m_vision;
  private final SwerveSubsystem m_swerve;

  /**
   * @param climber The subsystem used by this command.
   */
  public VisionCommand(VisionSubsystem vision, SwerveSubsystem swerve) {
    this.m_vision = vision;
    this.m_swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isAutonomous()) {
      var opRobotPose = m_vision.getEstimatedGlobalPose();
      opRobotPose.ifPresent(
          est -> {
            Pose2d estPose = est.estimatedPose.toPose2d();
            var stdDevs = m_vision.getEstimationStdDevs(estPose);

            m_swerve.addVisionMeasurement(estPose, est.timestampSeconds, stdDevs);

            if (DEBUGGING) {
              System.out.println("Got New Position");
              SmartDashboard.putNumber("estPoseX", estPose.getX());
              SmartDashboard.putNumber("estPoseY", estPose.getY());
              SmartDashboard.putNumber("estPoseRotation", estPose.getRotation().getDegrees());
            }
          });
    }
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
