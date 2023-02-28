package frc.robot.autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class driveToTrajectory extends SequentialCommandGroup {
  public driveToTrajectory(Drivetrain driveTrain, Trajectory trajectory) {
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            driveTrain::getPose,
            driveTrain.kinematics,
            driveTrain.getAutoXController(),
            driveTrain.getAutoYController(),
            driveTrain.getAutoProfiledThetaController(),
            driveTrain::setSwerveModuleStates,
            driveTrain);

    addCommands(
        Commands.runOnce(() -> driveTrain.resetOdometry(trajectory.getInitialPose()), driveTrain),
        swerveControllerCommand);
  }
}
