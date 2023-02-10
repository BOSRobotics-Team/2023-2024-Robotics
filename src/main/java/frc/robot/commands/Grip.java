package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;

public class Grip extends InstantCommand {

  private final Arm m_arm;
  private final boolean m_grip;

  public Grip(Arm arm, boolean grip) {
    m_arm = arm;
    m_grip = grip;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_grip) {
      m_arm.gripOpen();
    } else {
      m_arm.gripClose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
