package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class driveToTrajectory extends SequentialCommandGroup {
    public driveToTrajectory(SwerveDriveTrain driveTrain, Trajectory trajectory){
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, SwerveDriveTrain.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                driveTrain::getPose,
                SwerveDriveTrain.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                driveTrain::setModuleStates,
                driveTrain);


        addCommands(
            new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}