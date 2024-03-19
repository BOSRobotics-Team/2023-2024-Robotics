package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ShooterSubsystem;

public class ShootSlowCommand extends Command {
  private final ShooterSubsystem m_shooter;

  private boolean m_isFinished = false;
  private int m_shootCounter = 0;

  public ShootSlowCommand(ShooterSubsystem shooter) {

    this.m_shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shootCounter = 80;
    m_isFinished = false;

    m_shooter.reverseintake();
    System.out.println("Shooting slooow");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_shootCounter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
