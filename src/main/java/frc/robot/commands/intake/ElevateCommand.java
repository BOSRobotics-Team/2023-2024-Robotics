package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ElevatorSubsystem;

public class ElevateCommand extends Command {
  private final ElevatorSubsystem m_elevator;

  private boolean m_isFinished = false;
  private double m_targetHeight = 0.0;

  public ElevateCommand(ElevatorSubsystem elevate, double height) {

    this.m_elevator = elevate;
    m_targetHeight = height;

    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_isFinished = false;

    m_elevator.setHeight(m_targetHeight);
    System.out.println("Elevator Height to " + m_targetHeight);
  }

  @Override
  public void execute() {
    m_isFinished = m_elevator.getOnTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
