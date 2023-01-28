package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArm extends CommandBase {

  private final Arm m_arm;
  private final double m_length;

  public ExtendArm(Arm arm, double length) {
    m_arm = arm;
    m_length = length;
    addRequirements(m_arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.extendArm(m_length);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
// Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getArmExtension() - m_length) < 0.01;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
