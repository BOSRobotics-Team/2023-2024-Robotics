package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class JustShootCommand extends Command {
  private final IntakeSubsystem m_intake;

  private boolean m_isFinished = false;
  private int m_shootCounter = 0;

  public JustShootCommand(IntakeSubsystem intake) {
    this.m_intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_shootCounter = 25;
    m_isFinished = false;
  }

  @Override
  public void execute() {

    m_intake.run();
    System.out.println("Piece Loaded - shooting");
    m_isFinished = (--m_shootCounter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
