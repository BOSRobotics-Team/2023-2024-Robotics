package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.gyro.GyroIO;
import frc.lib.swerve.SwerveModule;
import frc.lib.swerve.SwerveModuleIOSim;
import frc.lib.swerve.SwerveModuleIOTalonFX;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.operator_interface.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drivetrain.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();

  /* Operator Interface */
  private OperatorInterface oi = new OperatorInterface() {};

  /* Subsystems */
  private final GyroIO gyro = new GyroIO(Constants.GYRO_ID, Constants.GYRO_CAN_BUS);
  private final SwerveDriveTrain driveTrain;
  private final Arm arm;

  private final TestChecklist test;

  /* Cameras */
  // public UsbCamera cam0;
  // public UsbCamera cam1;

  /* Auto paths */
  public static Map<String, Trajectory> trajectoryList = new HashMap<String, Trajectory>();
  public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   if (RobotBase.isReal()) {
      // cam0 = CameraServer.startAutomaticCapture(0);
      // cam1 = CameraServer.startAutomaticCapture(1);

      // cam0.setConnectVerbose(0);
      // cam1.setConnectVerbose(0);

      SwerveModule flModule = new SwerveModule(new SwerveModuleIOTalonFX(DriveTrainConstants.mod0));
      SwerveModule frModule = new SwerveModule(new SwerveModuleIOTalonFX(DriveTrainConstants.mod1));
      SwerveModule blModule = new SwerveModule(new SwerveModuleIOTalonFX(DriveTrainConstants.mod2));
      SwerveModule brModule = new SwerveModule(new SwerveModuleIOTalonFX(DriveTrainConstants.mod3));

      driveTrain = new SwerveDriveTrain(gyro, flModule, frModule, blModule, brModule);
      arm = new Arm();
   } else {
      SwerveModule flModule = new SwerveModule(new SwerveModuleIOSim(0));
      SwerveModule frModule = new SwerveModule(new SwerveModuleIOSim(1));
      SwerveModule blModule = new SwerveModule(new SwerveModuleIOSim(2));
      SwerveModule brModule = new SwerveModule(new SwerveModuleIOSim(3));

      driveTrain = new SwerveDriveTrain(gyro, flModule, frModule, blModule, brModule);
      arm = new Arm(); // use ArmSim later
   }
   test = new TestChecklist(gyro, driveTrain);

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();
    configureAutoCommands();
    configureAutoPaths();
  }

  public void logPeriodic() {
    driveTrain.logPeriodic();
    arm.logPeriodic();
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
            oi::getDriveScaling
        )
    );

    arm.setDefaultCommand(
      new TeleopArm(arm, oi::getArmLift, oi::getArmExtend)
    );

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {

    // Add commands to Autonomous Sendable Chooser
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    
    // SmartDashboard Buttons
    SmartDashboard.putData("Auto mode", chooser);
    SmartDashboard.putData("RaiseArm 0.0", new RaiseArm(arm, 0.0));
    SmartDashboard.putData("RaiseArm 1.0", new RaiseArm(arm, 1.0));
    SmartDashboard.putData("RaiseArm 2.0", new RaiseArm(arm, 2.0));
    SmartDashboard.putData("ExtendArm 0.0", new ExtendArm(arm, 0.0));
    SmartDashboard.putData("ExtendArm 1.0", new ExtendArm(arm, 1.0));
    SmartDashboard.putData("ExtendArm 2.0", new ExtendArm(arm, 2.0));
    Shuffleboard.getTab("MAIN").add(chooser);
  }

  private void configureAutoPaths() {
    try {
      DirectoryStream<Path> stream = Files.newDirectoryStream(Robot.RESOURCES_PATH.resolve("paths"));
      for (Path file : stream) {
        if (!Files.isDirectory(file)) {
          trajectoryList.put(file.getFileName().toString().replaceFirst("[.][^.]+$", ""), TrajectoryUtil.fromPathweaverJson(file));
        }
      }
    }  catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());
   }
   chooser.addOption("Autonomous Command", new exampleAuto(driveTrain));
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
          AUTO_EVENT_MAP)
      );
    // demonstration of PathPlanner path group with event markers
    chooser.addOption("Test Path", autoTest);
  }

  public void testInit() {
    this.test.enableChecklist();
  }

  public void testPeriodic() {
    this.test.update();
   }
 
  public void testExit() {
    this.test.disableChecklist();
  }
}
