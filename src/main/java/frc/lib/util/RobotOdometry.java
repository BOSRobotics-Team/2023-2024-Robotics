package frc.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator;
  private SwerveModulePosition[] defaultPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private RobotOdometry() {
    estimator =
        new SwerveDrivePoseEstimator(
            DriveTrainConstants.swerveKinematics, new Rotation2d(), defaultPositions, new Pose2d());
  }

  public Pose2d getEstimatedPosition() {
    // if (this.customOdometry == null) {
    return this.estimator.getEstimatedPosition();
    // } else {
    // return this.customOdometry.getEstimatedPosition();
    // }
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    // if (this.customOdometry == null) {
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    // } else {
    //   this.customOdometry.resetPosition(gyroAngle, modulePositions, poseMeters);
    // }
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    // if (this.customOdometry == null) {
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    // } else {
    //   return this.customOdometry.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    // }
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // if (this.customOdometry == null) {
    this.estimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // } else {
    //   this.customOdometry.addVisionMeasurement(
    //       visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // }
  }

  // public void setCustomOdometry(DrivetrainIOCTRE customOdometry) {
  //   this.customOdometry = customOdometry;
  // }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return estimator;
  }
}
