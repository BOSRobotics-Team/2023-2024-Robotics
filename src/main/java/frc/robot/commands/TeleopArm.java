package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class TeleopArm extends Command {
  private Arm arm;
  private DoubleSupplier liftSup;
  private DoubleSupplier extendSup;

  public TeleopArm(Arm arm, DoubleSupplier liftSup, DoubleSupplier extendSup) {
    this.arm = arm;
    addRequirements(arm);

    this.liftSup = liftSup;
    this.extendSup = extendSup;
  }

  @Override
  public void execute() {
    /* Get Values */
    double liftVal = liftSup.getAsDouble();
    double extendVal = extendSup.getAsDouble();

    arm.teleop(liftVal, extendVal);
  }
}
