// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants.ClimberConstants;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ClimberSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_leftClimberMotor =
      new SimableCANSparkMax(ClimberConstants.LEFTCLIMBER_ID, MotorType.kBrushless);
  private final RelativeEncoder m_leftClimberEncoder;
  private final SparkLimitSwitch m_leftClimberLimit;

  private final SimableCANSparkMax m_rightClimberMotor =
      new SimableCANSparkMax(ClimberConstants.RIGHTCLIMBER_ID, MotorType.kBrushless);
  private final RelativeEncoder m_rightClimberEncoder;
  private final SparkLimitSwitch m_rightClimberLimit;

  private ISimWrapper mLeftSim;
  private ISimWrapper mRightSim;

  private boolean m_isLReset = false;
  private boolean m_isRReset = false;

  public ClimberSubsystem() {

    m_leftClimberMotor.restoreFactoryDefaults();
    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    m_leftClimberLimit =
        m_leftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_leftClimberMotor.setInverted(false);
    m_leftClimberMotor.setIdleMode(IdleMode.kBrake);
    m_leftClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_leftClimberMotor.setSoftLimit(
        SoftLimitDirection.kForward, ClimberConstants.kLClimberMaxHeight);

    m_rightClimberMotor.restoreFactoryDefaults();
    m_rightClimberEncoder = m_rightClimberMotor.getEncoder();
    m_rightClimberLimit =
        m_rightClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_rightClimberMotor.setInverted(false);
    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
    m_rightClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_rightClimberMotor.setSoftLimit(
        SoftLimitDirection.kForward, ClimberConstants.kRClimberMaxHeight);

    if (RobotBase.isSimulation()) {
      mLeftSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNEO(1),
                  ClimberConstants.kClimberGearRatio,
                  3.0,
                  Units.inchesToMeters(1.0),
                  0.0,
                  550.0,
                  false,
                  0.0),
              new RevMotorControllerSimWrapper(m_leftClimberMotor),
              RevEncoderSimWrapper.create(m_leftClimberMotor));
      mRightSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNEO(1),
                  ClimberConstants.kClimberGearRatio,
                  3.0,
                  Units.inchesToMeters(1.0),
                  0.0,
                  550.0,
                  false,
                  0.0),
              new RevMotorControllerSimWrapper(m_rightClimberMotor),
              RevEncoderSimWrapper.create(m_rightClimberMotor));
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
    /*m_isLReset && */ isLClimbMinLimitSwitch()) {
      m_leftClimberEncoder.setPosition(0);
      // m_leftClimberMotor.set(0.0);
      // m_isLReset = false;
    }
    if (
    /*m_isRReset && */ isRClimbMinLimitSwitch()) {
      m_rightClimberEncoder.setPosition(0);
      // m_rightClimberMotor.set(0.0);
      // m_isRReset = false;
    }
    updateSmartDashboard();
  }

  public double getLClimberPosition() {
    return m_leftClimberEncoder.getPosition();
  }

  public double getRClimberPosition() {
    return m_rightClimberEncoder.getPosition();
  }

  public boolean isResetting() {
    return m_isLReset || m_isRReset;
  }

  public boolean isLClimbMinLimitSwitch() {
    return m_leftClimberLimit.isPressed();
  }

  public boolean isRClimbMinLimitSwitch() {
    return m_rightClimberLimit.isPressed();
  }

  public void teleop(double lval, double rval) {
    if (!isResetting()) {
      // m_leftClimberMotor.set(MathUtil.applyDeadband(lval, STICK_DEADBAND));
      // m_rightClimberMotor.set(MathUtil.applyDeadband(rval, STICK_DEADBAND));
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("LClimber Postion", m_leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("RClimber Postion", m_rightClimberEncoder.getPosition());
    SmartDashboard.putBoolean("LClimber Limit", isLClimbMinLimitSwitch());
    SmartDashboard.putBoolean("RClimber Limit", isRClimbMinLimitSwitch());
  }
}
