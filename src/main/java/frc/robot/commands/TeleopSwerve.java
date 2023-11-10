package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Drivetrain driveTrain;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private DoubleSupplier scaleFactorSup;
  private DoubleSupplier rotateFactorSup;

  public TeleopSwerve(
      Drivetrain driveTrain,
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
    double deadBand = STICK_DEADBAND;
    double maxSpeed = DriveTrainConstants.maxSpeed * scaleFactorSup.getAsDouble();
    double maxRotate = DriveTrainConstants.maxAngularVelocity * rotateFactorSup.getAsDouble();

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
