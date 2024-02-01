// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class TeleopClimber extends Command {

  private final ClimberSubsystem m_climber;
  private final DoubleSupplier m_vL;
  private final DoubleSupplier m_vR;

  /**
   * @param climber The subsystem used by this command.
   */
  public TeleopClimber(
      ClimberSubsystem climber,
      DoubleSupplier vL,
      DoubleSupplier vR) {
    this.m_climber = climber;
    this.m_vL = vL;
    this.m_vR = vR;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lVel = m_vL.getAsDouble();
    double rVel = m_vR.getAsDouble();

    if (DEBUGGING) {
      SmartDashboard.putNumber("lVel", lVel);
      SmartDashboard.putNumber("rVel", rVel);
    }

    // Climb using raw values.
    m_climber.teleop(lVel, rVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
