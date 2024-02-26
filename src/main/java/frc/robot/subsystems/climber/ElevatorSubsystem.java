// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Change to elevator for shooter with NEOs

package frc.robot.subsystems.climber;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ElevatorSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_leftElevatorMotor =
      new SimableCANSparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder m_leftElevatorEncoder;
  private final SparkLimitSwitch m_leftElevatorLimit;

  private final SimableCANSparkMax m_rightElevatorMotor =
      new SimableCANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder m_rightElevatorEncoder;
  private final SparkLimitSwitch m_rightElevatorLimit;

  private ISimWrapper mLeftSim;
  private ISimWrapper mRightSim;

  private boolean m_isLReset = false;
  private boolean m_isRReset = false;

  public ElevatorSubsystem() {

    m_leftElevatorMotor.restoreFactoryDefaults();
    m_leftElevatorEncoder = m_leftElevatorMotor.getEncoder();
    m_leftElevatorLimit =
        m_leftElevatorMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_leftElevatorMotor.setInverted(false);
    m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_leftElevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_leftElevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 20);

    m_rightElevatorMotor.restoreFactoryDefaults();
    m_rightElevatorEncoder = m_rightElevatorMotor.getEncoder();
    m_rightElevatorLimit =
        m_rightElevatorMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_rightElevatorMotor.setInverted(false);
    m_rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_rightElevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_rightElevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 20);

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
    if (
    /*m_isLReset && */ isLElevatorMinLimitSwitch()) {
      m_leftElevatorEncoder.setPosition(0);
      // m_leftElevatorMotor.set(0.0);
      // m_isLReset = false;
    }
    if (
    /*m_isRReset && */ isRElevatorMinLimitSwitch()) {
      m_rightElevatorEncoder.setPosition(0);
      // m_rightElevatorMotor.set(0.0);
      // m_isRReset = false;
    }
    updateSmartDashboard();
  }

  public double getLElevatorPosition() {
    return m_leftElevatorEncoder.getPosition();
  }

  public double getRElevatorPosition() {
    return m_rightElevatorEncoder.getPosition();
  }

  public boolean isResetting() {
    return m_isLReset || m_isRReset;
  }

  public boolean isLElevatorMinLimitSwitch() {
    return m_leftElevatorLimit.isPressed();
  }

  public boolean isRElevatorMinLimitSwitch() {
    return m_rightElevatorLimit.isPressed();
  }

  public void teleop(double lval, double rval) {
    if (!isResetting()) {
      // m_leftElevatorMotor.set(MathUtil.applyDeadband(lval, STICK_DEADBAND));
      // m_rightElevatorMotor.set(MathUtil.applyDeadband(rval, STICK_DEADBAND));
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("LClimber Postion", m_leftElevatorEncoder.getPosition());
    SmartDashboard.putNumber("RClimber Postion", m_rightElevatorEncoder.getPosition());
    SmartDashboard.putBoolean("LClimber Limit", isLElevatorMinLimitSwitch());
    SmartDashboard.putBoolean("RClimber Limit", isRElevatorMinLimitSwitch());
  }
}
