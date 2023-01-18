package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> chooser = new SendableChooser<>();

  private final boolean useXBoxController = true;

  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private int translationAxis = Joystick.AxisType.kY.value;
  private int strafeAxis = Joystick.AxisType.kX.value;
  private int rotationAxis = Joystick.AxisType.kTwist.value;

  private double scaleFactor = 0.5;
  private boolean updateScale = false;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

// Camera
  // public UsbCamera cam0;
  // public UsbCamera cam1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (useXBoxController) {
      translationAxis = XboxController.Axis.kLeftY.value; 
      strafeAxis = XboxController.Axis.kLeftX.value;
      rotationAxis = XboxController.Axis.kRightX.value;
    }
  
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> driver.getRawAxis(translationAxis), 
            () -> driver.getRawAxis(strafeAxis), 
            () -> driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean(),
            () -> this.getScalingFactor()
        )
    );

    // Configure the button bindings
    configureButtonBindings();

        // Add commands to Autonomous Sendable Chooser
    chooser.setDefaultOption("Autonomous Command", new exampleAuto(s_Swerve));
    // chooser.addOption("AutoDriveStraight Command", m_autoDriveStraightCommand);
    // chooser.addOption("AutoDriveTurn Command", m_autoDriveTurnCommand);

    // SmartDashboard Buttons
    SmartDashboard.putData("Auto mode", chooser);
    SmartDashboard.putData("Autonomous Command", new exampleAuto(s_Swerve));
    // SmartDashboard.putData("Autonomous AutoDriveStraight", m_autoDriveStraightCommand);
    // SmartDashboard.putData("Autonomous AutoDriveTurn", m_autoDriveTurnCommand);
    // SmartDashboard.putData("CommandDriveTrain", m_cmdDriveTrainCommand);

    for (Map.Entry<String, Trajectory> entry : Robot.trajectoryList.entrySet()) {
      chooser.addOption(entry.getKey(), new driveToTrajectory(s_Swerve, entry.getValue()));
      SmartDashboard.putData(entry.getKey(), new driveToTrajectory(s_Swerve, entry.getValue()));
    }

    if (RobotBase.isReal()) {
      // cam0 = CameraServer.startAutomaticCapture(0);
      // cam1 = CameraServer.startAutomaticCapture(1);

      // cam0.setConnectVerbose(0);
      // cam1.setConnectVerbose(0);
    }
  }

  public void logPeriodic() {
    s_Swerve.logPeriodic();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public double getScalingFactor() {
    if (useXBoxController) {
      int povVal = driver.getPOV();

      if (updateScale && povVal == -1) {
          updateScale = false;
      }
      if (!updateScale && povVal == 0) {
        scaleFactor += 0.05;
        System.out.println("Setting scaleFactor to " + scaleFactor);
        updateScale = true;
      } else if (!updateScale && povVal == 180) {
        scaleFactor -= 0.05;
        System.out.println("Setting scaleFactor to " + scaleFactor);
        updateScale = true;
      }
    } else {
      scaleFactor = driver.getThrottle();
    }
    scaleFactor = MathUtil.clamp(scaleFactor, 0.1, 1.0);

    return scaleFactor;
  }
}
