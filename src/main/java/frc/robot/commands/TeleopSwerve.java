package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private SwerveDriveTrain driveTrain;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private DoubleSupplier scaleFactorSup;
  private DoubleSupplier rotateFactorSup;

  public TeleopSwerve(
      SwerveDriveTrain driveTrain,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      DoubleSupplier scaleFactorSup,
      DoubleSupplier rotateFactorSub) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.scaleFactorSup = scaleFactorSup;
    this.rotateFactorSup = rotateFactorSub;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double deadBand = RobotPreferences.stickDeadband.get();
    double maxSpeed = RobotPreferences.Swerve.maxSpeed.get() * scaleFactorSup.getAsDouble();
    double maxRotate = RobotPreferences.Swerve.maxAngularVelocity.get() * rotateFactorSup.getAsDouble();

    double translationVal = this.scaleController(translationSup.getAsDouble(), deadBand) * maxSpeed;
    double strafeVal = this.scaleController(strafeSup.getAsDouble(), deadBand) * maxSpeed;
    double rotationVal = this.scaleController(rotationSup.getAsDouble(), deadBand) * maxRotate;

    /* Drive */
    driveTrain.drive(translationVal, strafeVal, rotationVal);
  }

  public double scaleController(double value, double deadBand) {
    value = MathUtil.applyDeadband(value, deadBand);
    // Square the axis - more natural for drivers
    return Math.copySign(value * value, value);
  }
}
