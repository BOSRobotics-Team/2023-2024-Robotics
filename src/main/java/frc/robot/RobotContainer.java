package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerTrajectory;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.gyro.*;
import frc.lib.limelightvision.LimelightHelpers;
import frc.lib.swerve.*;
import frc.robot.commands.*;
import frc.robot.operator_interface.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drivetrain.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  /* Operator Interface */
  public OperatorInterface oi = new OperatorInterface() {};

  /* Subsystems */
  public final PowerDistribution power = new PowerDistribution();
  public final GyroIO gyro = new GyroIOPigeon2(Constants.GYRO_ID, Constants.GYRO_CAN_BUS);
  public final Drivetrain driveTrain;
  public final Arm arm;

  /* Cameras */
  // public UsbCamera cam0;

  /* Auto paths */
  //  public SwerveAutoBuilder autoBuilder;

  public static Map<String, Trajectory> trajectoryList = new HashMap<String, Trajectory>();
  public static Map<String, List<PathPlannerTrajectory>> pptrajectoryList =
      new HashMap<String, List<PathPlannerTrajectory>>();
  public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

  private static RobotContainer instance;
  public static RobotContainer GetInstance(){ return instance; }
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    instance = this;

    SwerveModuleIO flModule;
    SwerveModuleIO frModule;
    SwerveModuleIO blModule;
    SwerveModuleIO brModule;

    if (RobotBase.isReal()) {
      // Make sure you only configure port forwarding once in your robot code.
      for (int port = 5800; port <= 5805; port++) {
        PortForwarder.add(port, Constants.LIMELIGHTURL, port);
      }

      flModule = new SwerveModuleIOTalonFX(DriveTrainConstants.mod0);
      frModule = new SwerveModuleIOTalonFX(DriveTrainConstants.mod1);
      blModule = new SwerveModuleIOTalonFX(DriveTrainConstants.mod2);
      brModule = new SwerveModuleIOTalonFX(DriveTrainConstants.mod3);
    } else {
      flModule = new SwerveModuleIOSim(DriveTrainConstants.mod0.moduleNumber);
      frModule = new SwerveModuleIOSim(DriveTrainConstants.mod1.moduleNumber);
      blModule = new SwerveModuleIOSim(DriveTrainConstants.mod2.moduleNumber);
      brModule = new SwerveModuleIOSim(DriveTrainConstants.mod3.moduleNumber);
    }
    driveTrain =
        new Drivetrain(
            gyro,
            new SwerveModule(flModule),
            new SwerveModule(frModule),
            new SwerveModule(blModule),
            new SwerveModule(brModule));
    arm = new Arm();

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    configureAutoCommands();
    configureAutoPaths();

    // cam0 = CameraServer.startAutomaticCapture(0);
    // cam0.setConnectVerbose(0);

    LimelightHelpers.setLEDMode_ForceOff(Constants.LIMELIGHTNAME); // setLEDMode_PipelineControl
    LimelightHelpers.setCameraMode_Driver(Constants.LIMELIGHTNAME); // setCameraMode_Processor
    LimelightHelpers.setStreamMode_Standard(Constants.LIMELIGHTNAME);
    // LimelightHelpers.setStreamMode_PiPMain("");
    // LimelightHelpers.setStreamMode_PiPSecondary("");

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    tab.add(autoChooser).withSize(2, 1);
    tab.addNumber("DriveTrain/Drive Scaling", () -> oi.getDriveScaling());
    tab.addNumber("DriveTrain/Rotate Scaling", () -> oi.getRotateScaling());
  
  }
  
  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    driveTrain.setDefaultCommand(
        new TeleopSwerve(
            driveTrain,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            oi::getDriveScaling,
            oi::getRotateScaling));

    arm.setDefaultCommand(new TeleopArm(arm, oi::getArmLift, oi::getArmExtend));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(driveTrain::zeroGyro, driveTrain));
    // Robot relative navigation
    oi.getRobotRelative().onTrue(Commands.runOnce(driveTrain::disableFieldRelative, driveTrain));
    oi.getRobotRelative().onFalse(Commands.runOnce(driveTrain::enableFieldRelative, driveTrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(driveTrain::enableXstance, driveTrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(driveTrain::disableXstance, driveTrain));

    oi.getGripToggle().onTrue(Commands.runOnce(arm::gripToggle, arm));

    oi.getArmCalibrate().onTrue(Commands.runOnce(arm::resetArm, arm));
    oi.getArmPosition0().onTrue(Commands.runOnce(() -> arm.setArmPosition(0), arm));
    oi.getArmPosition1().onTrue(Commands.runOnce(() -> arm.setArmPosition(1), arm));
    oi.getArmPosition2().onTrue(Commands.runOnce(() -> arm.setArmPosition(2), arm));
    oi.getArmPosition3().onTrue(Commands.runOnce(() -> arm.setArmPosition(3), arm));
    oi.getArmTargetToggle()
        .onTrue(Commands.runOnce(() -> arm.targetCones(!arm.isTargetCone()), arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {

    // Add commands to Autonomous Sendable Chooser
    autoChooser = AutoBuilder.buildAutoChooser();

    // SmartDashboard Buttons
    SmartDashboard.putData("Auto chooser", autoChooser);
    // SmartDashboard.putData("Calibrate Arm", Commands.runOnce(arm::resetArm, arm));
  }
  private void configureAutoPaths() {
    for (int pos = 1; pos <= 3; ++pos) {
      NamedCommands.registerCommand("CubePosition" + pos, Commands.sequence(
              Commands.runOnce(() -> arm.targetCones(false), arm), new PositionArm(arm, pos)));
    }
    for (int pos = 1; pos <= 3; ++pos) {
      NamedCommands.registerCommand("ConePosition" + pos, Commands.sequence(
              Commands.runOnce(() -> arm.targetCones(true), arm), new PositionArm(arm, pos)));
    }
    NamedCommands.registerCommand("DropPiece", new Grip(arm, true));
    NamedCommands.registerCommand("GrabPiece", new Grip(arm, false));
    NamedCommands.registerCommand("ZeroArm", new PositionArm(arm, 0));

/*    try {
      DirectoryStream<Path> stream =
          Files.newDirectoryStream(Robot.RESOURCES_PATH.resolve("pathplanner"));
      for (Path file : stream) {
        if (!Files.isDirectory(file)) {
          String name = file.getFileName().toString().replaceFirst("[.][^.]+$", "");
          pptrajectoryList.put(
              name,
              PathPlanner.loadPathGroup(
                  name,
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        }
      }
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open pptrajectory: ", ex.getStackTrace());
    }

    for (Map.Entry<String, List<PathPlannerTrajectory>> entry : pptrajectoryList.entrySet()) {
      chooser.addOption(entry.getKey(), autoBuilder.fullAuto(entry.getValue()));
    } */
    
    // for (Map.Entry<String, PathPlannerTrajectory> entry : pptrajectoryList.entrySet()) {
    //   Command autoPathBlue =
    //       new FollowPathWithEvents(
    //           new FollowPath(entry.getValue(), driveTrain, true),
    //           entry.getValue().getMarkers(),
    //           AUTO_EVENT_MAP);
    //   Command autoPathRed =
    //     new FollowPathWithEvents(
    //         new FollowPath(PathPlannerTrajectory.transformTrajectoryForAlliance(entry.getValue(),
    // Alliance.Red), driveTrain, true),
    //         entry.getValue().getMarkers(),
    //         AUTO_EVENT_MAP);
    //   chooser.addOption(entry.getKey(), Commands.either(autoPathBlue, autoPathRed, () ->
    // DriverStation.getAlliance() == Alliance.Blue));
    // }
    //chooser.addOption("Autonomous Command", new exampleAuto(driveTrain));

    /*    try {
      DirectoryStream<Path> stream =
          Files.newDirectoryStream(Robot.RESOURCES_PATH.resolve("paths"));
      for (Path file : stream) {
        if (!Files.isDirectory(file)) {
          trajectoryList.put(
              file.getFileName().toString().replaceFirst("[.][^.]+$", ""),
              TrajectoryUtil.fromPathweaverJson(file));
        }
      }
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());
    }
    for (Map.Entry<String, Trajectory> entry : trajectoryList.entrySet()) {
      chooser.addOption(entry.getKey(), new driveToTrajectory(driveTrain, entry.getValue()));
    }

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "testPaths1",
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), driveTrain, true),
                auto1Paths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(driveTrain::enableXstance, driveTrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(driveTrain::disableXstance, driveTrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), driveTrain, false),
                auto1Paths.get(1).getMarkers(),
                AUTO_EVENT_MAP));
    // demonstration of PathPlanner path group with event markers
    chooser.addOption("Test Path", autoTest); */
  }

  public void simulationInit() {}
  public void simulationPeriodic() {}

  public void testInit() {
    
  }
  public void testPeriodic() {
    
  }
  public void testExit() {
    
  }

}
