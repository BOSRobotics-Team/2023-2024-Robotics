package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class RaiseArm extends Command {

  private final Arm m_arm;
  private final double m_pctHeight;

  public RaiseArm(Arm arm, double pctHeight) {
    m_arm = arm;
    m_pctHeight = pctHeight;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.raiseArm(m_pctHeight);
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
    return m_arm.isArmRaised();
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
