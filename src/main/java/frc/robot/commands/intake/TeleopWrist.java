// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.WristSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class TeleopWrist extends Command {

  private final WristSubsystem m_wrist;
  private final DoubleSupplier m_vHt;

  /**
   * @param climber The subsystem used by this command.
   */
  public TeleopWrist(WristSubsystem wrist, DoubleSupplier vHt) {
    this.m_wrist = wrist;
    this.m_vHt = vHt;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double hVel = m_vHt.getAsDouble();

    if (DEBUGGING) {
      SmartDashboard.putNumber("Wrist hVel", hVel);
    }
    m_wrist.teleop(hVel);
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
