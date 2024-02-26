package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ShooterSubsystem;

public class IntakeCommand extends Command {
  private final ShooterSubsystem m_shooter;

  private boolean m_isFinished = false;
  private int m_shootCounter = 0;

  public IntakeCommand(ShooterSubsystem shooter) {

    this.m_shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shootCounter = 25;
    m_isFinished = false;

    m_shooter.intake();
    System.out.println("Intake started");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_shootCounter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    System.out.println("Intake stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
