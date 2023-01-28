package frc.robot;

import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.DriveGyro;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.operator_interface.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> chooser = new SendableChooser<>();

  /* Operator Interface */
  private OperatorInterface oi = new OperatorInterface() {};

  /* Subsystems */
  private final DriveGyro gyro = new DriveGyro(Constants.GYRO_ID, Constants.GYRO_CAN_BUS);
  private final SwerveDriveTrain driveTrain = new SwerveDriveTrain(gyro);
  private final Arm arm = new Arm();

  /* Cameras */
  // public UsbCamera cam0;
  // public UsbCamera cam1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   if (RobotBase.isReal()) {
      // cam0 = CameraServer.startAutomaticCapture(0);
      // cam1 = CameraServer.startAutomaticCapture(1);

      // cam0.setConnectVerbose(0);
      // cam1.setConnectVerbose(0);
    }

    updateOI();

    configureAutoCommands();
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
    oi.getRobotRelative().onTrue(Commands.runOnce(driveTrain::enableFieldRelative, driveTrain));
    oi.getRobotRelative().onFalse(Commands.runOnce(driveTrain::disableFieldRelative, driveTrain));

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
    chooser.addOption("Autonomous Command", new exampleAuto(driveTrain));
    for (Map.Entry<String, Trajectory> entry : Robot.trajectoryList.entrySet()) {
      chooser.addOption(entry.getKey(), new driveToTrajectory(driveTrain, entry.getValue()));
    }
    
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
}
