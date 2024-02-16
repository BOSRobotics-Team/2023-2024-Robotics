package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ShooterSubsystem;

public class SpinUpShootersCommand extends Command {
  private final ShooterSubsystem m_shooter;

  private boolean m_isFinished = false;

  public SpinUpShootersCommand(ShooterSubsystem shooter) {
    this.m_shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_shooter.run();
  }

  @Override
  public void execute() {

    if (m_shooter.isOnTargetAverage(7)) {
      m_isFinished = true;
      System.out.println("Shooters ready to shoot");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
