package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intake;

  private boolean m_pieceLoaded = false;

  public IntakeCommand(IntakeSubsystem intake) {
    this.m_intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_pieceLoaded = m_intake.get_intakeSensor();
    if (!m_pieceLoaded) m_intake.run();
  }

  @Override
  public void execute() {
    if (m_intake.get_intakeSensor()) {
      // Disable the intake motors
      m_intake.stop();
      m_pieceLoaded = true;
      System.out.println("Piece Loaded - stop intake");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pieceLoaded;
  }
}
