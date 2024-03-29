package frc.robot.subsystems.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  // Create a vision photon camera
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName1);
  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  // Camera result for vision camera
  private PhotonPipelineResult cameraResult;

  // Subsystem Constructor
  public VisionSubsystem() {

    this.photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            VisionConstants.kRobotToCam1);

    this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      this.visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      this.visionSim.addAprilTags(VisionConstants.kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      this.cameraSim = new PhotonCameraSim(camera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      this.visionSim.addCamera(cameraSim, VisionConstants.kRobotToCam1);

      this.cameraSim.enableDrawWireframe(true);
    } else {
      // Port forward photon vision so we can access it with an ethernet cable
      // Make sure you only configure port forwarding once in your robot code.
      for (int port = 5800; port <= 5805; port++) {
        PortForwarder.add(port, VisionConstants.PHOTONVISIONURL, port);
      }
    }

    // Update camera results before periodic
    updateCameraResults();
    camera.setLED(VisionLEDMode.kDefault);

    // LimelightHelpers.setLEDMode_ForceOff(Constants.LIMELIGHTNAME); // setLEDMode_PipelineControl
    // LimelightHelpers.setCameraMode_Driver(Constants.LIMELIGHTNAME); // setCameraMode_Processor
    // LimelightHelpers.setStreamMode_Standard(Constants.LIMELIGHTNAME);
    // LimelightHelpers.setStreamMode_PiPMain("");
    // LimelightHelpers.setStreamMode_PiPSecondary("");

  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = cameraResult.getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = cameraResult.getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonEstimator.setReferencePose(prevEstimatedRobotPose);
    return getEstimatedGlobalPose();
  }

  // Update the camera results
  private void updateCameraResults() {
    cameraResult = camera.getLatestResult();
  }

  // Checks if camera sees targets, must use to stop null exeptions!
  public Boolean hasTargets() {
    return (cameraResult.hasTargets());
  }

  // Returns the single best target from the camera
  public PhotonTrackedTarget getBestTarget() {
    return (cameraResult.getBestTarget());
  }

  // Returns all targets from the camera in an array
  public List<PhotonTrackedTarget> getTargetList() {
    return (cameraResult.getTargets());
  }

  // Returns a picked target from the target list
  public PhotonTrackedTarget getTargetFromList(int num) {
    return (cameraResult.getTargets().get(num));
  }

  // Returns the size of the target list
  public int getListSize() {
    return (cameraResult.getTargets().size());
  }

  // Returns a percentage of how much area a target takes up, 0 - 100 percent
  public double getTargetArea() {
    return (getBestTarget().getArea());
  }

  public double getTargetPitch() {
    return (getBestTarget().getPitch());
  }

  // Returns the april tag ID number
  public int getTargetID() {
    return (getBestTarget().getFiducialId());
  }

  public double getTargetDistance() {
    // Camera height M, Target height M, Camera pitch R, Target pitch R.
    return (PhotonUtils.calculateDistanceToTargetMeters(0.774, 0.361, 0.0174, getTargetPitch()));
  }

  public Transform3d getTargetTransform() {
    return (getBestTarget().getBestCameraToTarget());
  }

  public Double getTargetTransformHeight() {
    return (getBestTarget().getBestCameraToTarget().getZ());
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    // Put targets? value
    SmartDashboard.putBoolean("Targets?", hasTargets());

    // Check if targets are found before putting values to prevent null!
    if (hasTargets()) {
      SmartDashboard.putString("Target ID", getTargetID() + "");
      SmartDashboard.putString("Target Pitch", getTargetPitch() + "");
      SmartDashboard.putString("Target Area", getTargetArea() + "%");
      SmartDashboard.putNumber("Target Distance X-Plane", getTargetTransform().getX());
    } else {
      SmartDashboard.putString("Target ID", "No ID Found!");
      SmartDashboard.putString("Target Pitch", "-1");
      SmartDashboard.putString("Target Area", "0" + "%");
      SmartDashboard.putNumber("Target Distance X-Plane", -1);
    }
    SmartDashboard.putString("LED State", camera.getLEDMode().toString());
  }

  public void setLEDOn() {
    camera.setLED(VisionLEDMode.kBlink);
    DriverStation.reportWarning("CHANGE LED", true);
  }

  // A periodic loop, updates smartdashboard and camera results
  @Override
  public void periodic() {
    // SmartDashboard.putString("Camera", camera.toString());
    updateCameraResults();
    updateSmartDashboard();
  }
}
