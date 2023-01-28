package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RaiseArm extends CommandBase {

  private final Arm m_arm;
  private final double m_height;

  public RaiseArm(Arm arm, double height) {
    m_arm = arm;
    m_height = height;
    addRequirements(m_arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.raiseArm(m_height);
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
    return Math.abs(m_arm.getArmHeight() - m_height) < 0.01;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
