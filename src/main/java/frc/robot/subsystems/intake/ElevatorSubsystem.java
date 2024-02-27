// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Change to elevator for shooter with NEOs

package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ElevatorSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_leftElevatorMotor =
      new SimableCANSparkMax(ElevatorConstants.LEFTELEVATOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_leftElevatorEncoder = m_leftElevatorMotor.getEncoder();

  private final SimableCANSparkMax m_rightElevatorMotor =
      new SimableCANSparkMax(ElevatorConstants.RIGHTELEVATOR_ID, MotorType.kBrushless);

  private final CANcoder m_canCoder = new CANcoder(ElevatorConstants.CANCODER_ID);

  private double m_targetHeight = 0.0;

  private ISimWrapper mLeftSim;
  private ISimWrapper mRightSim;

  public ElevatorSubsystem() {

    m_leftElevatorMotor.restoreFactoryDefaults();
    m_leftElevatorMotor.setInverted(false);
    m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);

    m_rightElevatorMotor.restoreFactoryDefaults();
    m_rightElevatorMotor.setInverted(true);
    m_rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_rightElevatorMotor.follow(m_leftElevatorMotor);

    m_leftElevatorEncoder.setPosition(m_canCoder.getPosition().getValue());

    if (RobotBase.isSimulation()) {
      mLeftSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNEO(1), 1, 3.0, Units.inchesToMeters(1.0), 0.0, 550.0, false, 0.0),
              new RevMotorControllerSimWrapper(m_leftElevatorMotor),
              RevEncoderSimWrapper.create(m_leftElevatorMotor));
      mRightSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNEO(1), 10, 3.0, Units.inchesToMeters(1.0), 0.0, 550.0, false, 0.0),
              new RevMotorControllerSimWrapper(m_rightElevatorMotor),
              RevEncoderSimWrapper.create(m_rightElevatorMotor));
    }
  }

  @Override
  public void simulationPeriodic() {
    mLeftSim.update();
    mRightSim.update();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    if (this.getPosition() <= 0.0) {
      m_leftElevatorMotor.stopMotor();
    }

    updateSmartDashboard();
  }

  public void setHeight(double ht) {
    m_targetHeight = ht;
  }

  public double getPosition() {
    return m_canCoder.getPosition().getValue();
  }

  public boolean getOnTarget() {
    return Math.abs(this.getPosition() - m_targetHeight) < 0.1;
  }

  public void teleop(double val) {
    m_leftElevatorMotor.set(MathUtil.applyDeadband(val, STICK_DEADBAND));
    // m_rightElevatorMotor.set(MathUtil.applyDeadband(rval, STICK_DEADBAND));
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("LClimber Postion", m_leftElevatorEncoder.getPosition());
    // SmartDashboard.putNumber("RClimber Postion", m_rightElevatorEncoder.getPosition());
  }
}
