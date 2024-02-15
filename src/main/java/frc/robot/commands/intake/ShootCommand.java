package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.ShooterSubsystem;

public class ShootCommand extends Command {
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  private boolean m_readyToShoot = false;
  private boolean m_isFinished = false;
  private int m_shootCounter = 0;

  public ShootCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.m_intake = intake;
    this.m_shooter = shooter;

    addRequirements(intake, shooter);
  }

  @Override
  public void initialize() {
    m_shootCounter = 25;
    m_readyToShoot = false;
    m_isFinished = false;

    if (m_intake.get_intakeSensor()) {
      m_intake.stop();
      m_shooter.run();
    } else {
      m_isFinished = true;
    }
  }

  @Override
  public void execute() {

    if (!m_readyToShoot) {
      if (m_shooter.isOnTarget()) {
        m_readyToShoot = true;
        System.out.println("Piece Loaded - ready to shoot");
      }
    }
    if (m_readyToShoot) {
      m_intake.run();
      System.out.println("Piece Loaded - shooting");
      m_isFinished = (--m_shootCounter <= 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
