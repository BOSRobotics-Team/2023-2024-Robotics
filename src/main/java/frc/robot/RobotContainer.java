package frc.robot;

//import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
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
import frc.robot.Constants.DriveTrainConstants;
// import frc.robot.commands.vision.VisionCommand;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
  // public final VisionSubsystem vision = new VisionSubsystem();

  public final SwerveSubsystem driveTrain =
      new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), DriveTrainConstants.swerveConfigurationName),
          DriveTrainConstants.swerveConfig,
          DriveTrainConstants.maxSpeed);

  // public final ShooterSubsystem shooter = new ShooterSubsystem();
  // public final WristSubsystem wrist = new WristSubsystem();
  // public final ElevatorSubsystem elevator = new ElevatorSubsystem();

  /* Test System */
  //  private TestChecklist m_test;

  /* Cameras */
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

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    configureAutoPaths();
    configureAutoCommands();

    // cam0 = CameraServer.startAutomaticCapture(0);
    // cam0.setConnectVerbose(0);

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    tab.add(autoChooser).withSize(2, 1);
    // tab.addNumber("DriveTrain/Drive Scaling", () -> oi.getDriveScaling());
    // tab.addNumber("DriveTrain/Rotate Scaling", () -> oi.getRotateScaling());

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

    configureButtonBindings();

    if (RobotBase.isSimulation()) {
      driveTrain.setDefaultCommand(
          driveTrain.simDriveCommand(oi::getTranslateX, oi::getTranslateY, oi::getRotate));
    } else {
      driveTrain.setDefaultCommand(
          driveTrain.driveCommand(
              oi::getTranslateX, oi::getTranslateY, oi::getRotate, oi::isRobotRelative));
    }
    // elevator.setDefaultCommand(new TeleopElevator(elevator, oi::getElevator));
    // wrist.setDefaultCommand(new TeleopWrist(wrist, oi::getWrist));
    // vision.setDefaultCommand(new VisionCommand(vision, driveTrain));
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
    oi.getXStanceButton()
        .onTrue(Commands.runOnce(driveTrain::enableXstance))
        .onFalse(Commands.runOnce(driveTrain::disableXstance));

    // oi.getRunIntake()
    //     .onTrue(Commands.runOnce(shooter::intake))
    //     .onFalse(Commands.runOnce(shooter::stop));
    // oi.getShoot().onTrue(Commands.runOnce(shooter::shoot)).onFalse(Commands.runOnce(shooter::stop));
    // oi.getYButton()
    //     .onTrue(Commands.runOnce(shooter::shoot2))
    //     .onFalse(Commands.runOnce(shooter::stop));

    // oi.getLeftBumper()
    //     .onTrue(Commands.runOnce(shooter::reverseintake))
    //     .onFalse(Commands.runOnce(shooter::stop));

    // oi.getElevatorUp().onTrue(Commands.runOnce(elevator::up));
    // oi.getElevatorDown().onTrue(Commands.runOnce(elevator::down));

    // oi.getWristUp().onTrue(Commands.runOnce(wrist::up));
    // oi.getWristDown().onTrue(Commands.runOnce(wrist::down));
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
  }

  private void configureAutoPaths() {
    // NamedCommands.registerCommand("Intake", new IntakeCommand(shooter));
    // NamedCommands.registerCommand(
    //     "ElevateUp", new ElevateCommand(elevator, ElevatorConstants.kTargetElevatorHigh));
    // NamedCommands.registerCommand(
    //     "ElevateDown", new ElevateCommand(elevator, ElevatorConstants.kTargetElevatorLow));
    // NamedCommands.registerCommand(
    //     "RotateUp", new RotateWristCommand(wrist, WristConstants.kTargetWristHigh));
    // NamedCommands.registerCommand(
    //     "RotateDown", new RotateWristCommand(wrist, WristConstants.kTargetWristLow));
    // NamedCommands.registerCommand("Shoot", new ShootCommand(shooter));
    // NamedCommands.registerCommand("ShootSlow", new ShootSlowCommand(shooter));
  }

  public void simulationInit() {}

  public void simulationPeriodic() {}

  public void testInit() {
    // m_test.initialize();
  }

  public void testPeriodic() {
    // m_test.periodic();
  }

  public void testExit() {
    // m_test.exit();
  }
}
