package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.limelightvision.LimelightHelpers;
import frc.robot.commands.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.operator_interface.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.testsystem.TestChecklist;
import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.photonvision.PhotonCamera;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;

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
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem driveTrain =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));

  //  public final Drivetrain driveTrain = new Drivetrain();
  public final Arm arm = new Arm();

  /* Test System */
  private TestChecklist m_test;

  /* Cameras */
  public PhotonCamera camera = new PhotonCamera("photonvision");
  // public UsbCamera cam0;

  public static Map<String, Trajectory> trajectoryList = new HashMap<String, Trajectory>();
  public static Map<String, List<PathPlannerTrajectory>> pptrajectoryList =
      new HashMap<String, List<PathPlannerTrajectory>>();
  public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

  private static RobotContainer instance;

  public static RobotContainer GetInstance() {
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    instance = this;

    if (RobotBase.isReal()) {
      // Make sure you only configure port forwarding once in your robot code.
      for (int port = 5800; port <= 5805; port++) {
        PortForwarder.add(port, Constants.LIMELIGHTURL, port);
      }
    }

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

    // m_test = new TestChecklist(driveTrain, arm);
    
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

    // AbsoluteDrive closedAbsoluteDrive =
    //     new AbsoluteDrive(
    //         driveTrain,
    //         oi::getTranslateX,
    //         oi::getTranslateY,
    //         oi::getRotate,
    //         () -> -driverXbox.getRightY());

    // AbsoluteFieldDrive closedFieldAbsoluteDrive =
    //     new AbsoluteFieldDrive(
    //         driveTrain,
    //         oi::getTranslateX,
    //         oi::getTranslateY,
    //         oi::getRotate);

    // AbsoluteDriveAdv closedAbsoluteDriveAdv =
    //     new AbsoluteDriveAdv(
    //         driveTrain,
    //         oi::getTranslateX,
    //         oi::getTranslateY,
    //         oi::getRotate,
    //         driverXbox::getYButtonPressed,
    //         driverXbox::getAButtonPressed,
    //         driverXbox::getXButtonPressed,
    //         driverXbox::getBButtonPressed);

    TeleopDrive teleopCmd =
        new TeleopDrive(
            driveTrain,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            oi::isRobotRelative,
            oi::getDriveScaling,
            oi::getRotateScaling);
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
    driveTrain.setDefaultCommand(teleopCmd);
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
    oi.getResetGyroButton().onTrue(Commands.runOnce(driveTrain::zeroGyro));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(driveTrain::enableXstance));
    oi.getXStanceButton().onFalse(Commands.runOnce(driveTrain::disableXstance));

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
      NamedCommands.registerCommand(
          "CubePosition" + pos,
          Commands.sequence(
              Commands.runOnce(() -> arm.targetCones(false), arm), new PositionArm(arm, pos)));
    }
    for (int pos = 1; pos <= 3; ++pos) {
      NamedCommands.registerCommand(
          "ConePosition" + pos,
          Commands.sequence(
              Commands.runOnce(() -> arm.targetCones(true), arm), new PositionArm(arm, pos)));
    }
    NamedCommands.registerCommand("DropPiece", new Grip(arm, true));
    NamedCommands.registerCommand("GrabPiece", new Grip(arm, false));
    NamedCommands.registerCommand("ZeroArm", new PositionArm(arm, 0));
  }

  public void simulationInit() {}

  public void simulationPeriodic() {}

  public void testInit() {
    m_test.initialize();
  }

  public void testPeriodic() {
    m_test.periodic();
  }

  public void testExit() {
    m_test.exit();
  }

}
