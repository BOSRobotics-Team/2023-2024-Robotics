package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ExtendArm extends Command {

  private final Arm m_arm;
  private final double m_pctLength;

  public ExtendArm(Arm arm, double pctLength) {
    m_arm = arm;
    m_pctLength = pctLength;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.extendArm(m_pctLength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return m_arm.isArmExtended();
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
