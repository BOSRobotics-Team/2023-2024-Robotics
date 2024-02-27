package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.WristSubsystem;

public class RotateWristCommand extends Command {
  private final WristSubsystem m_wrist;

  private boolean m_isFinished = false;
  private double m_targetPosition = 0.0;

  public RotateWristCommand(WristSubsystem wrist, double position) {

    this.m_wrist = wrist;
    m_targetPosition = position;

    addRequirements(m_wrist);
  }

  @Override
  public void initialize() {
    m_isFinished = false;

    m_wrist.setPosition(m_targetPosition);
    System.out.println("Wrist Position to " + m_targetPosition);
  }

  @Override
  public void execute() {
    m_isFinished = m_wrist.getOnTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
