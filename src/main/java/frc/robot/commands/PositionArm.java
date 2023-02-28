package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class PositionArm extends CommandBase {

  private final Arm m_arm;
  private final int m_position;

  public PositionArm(Arm arm, int position) {
    m_arm = arm;
    m_position = position;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmPosition(m_position);
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
    return !m_arm.isArmRaised() && !m_arm.isArmExtended();
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
