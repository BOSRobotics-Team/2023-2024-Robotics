package frc.robot.autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;

public class driveToTrajectory extends SequentialCommandGroup {
  public driveToTrajectory(SwerveDriveTrain driveTrain, Trajectory trajectory) {
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            driveTrain::getPose,
            SwerveDriveTrain.swerveKinematics,
            driveTrain.getAutoXController(),
            driveTrain.getAutoYController(),
            driveTrain.getAutoProfiledThetaController(),
            driveTrain::setModuleStates,
            driveTrain);

    addCommands(
        new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand);
  }
}
