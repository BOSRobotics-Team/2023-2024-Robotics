// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ClimberSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_leftClimberMotor =
      new SimableCANSparkMax(ClimberConstants.LEFTCLIMBER_ID, MotorType.kBrushless);
  private final RelativeEncoder m_leftClimberEncoder;
  private final SparkPIDController m_leftClimberController;
  private final SparkLimitSwitch m_leftClimberLimit;

  private final SimableCANSparkMax m_rightClimberMotor =
      new SimableCANSparkMax(ClimberConstants.RIGHTCLIMBER_ID, MotorType.kBrushless);
  private final RelativeEncoder m_rightClimberEncoder;
  private final SparkPIDController m_rightClimberController;
  private final SparkLimitSwitch m_rightClimberLimit;

  private ISimWrapper mLeftSim;
  private ISimWrapper mRightSim;

  private boolean m_isLReset = false;
  private boolean m_isRReset = false;

  private double m_LTargetSetpoint = 0;
  private double m_RTargetSetpoint = 0;

  // private float _lClimberMaxHeight = ClimberConstants.kLClimberMaxHeight;
  // private float _rClimberMaxHeight = ClimberConstants.kRClimberMaxHeight;

  public ClimberSubsystem() {

    m_leftClimberMotor.restoreFactoryDefaults();
    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    m_leftClimberController = m_leftClimberMotor.getPIDController();
    m_leftClimberLimit =
        m_leftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_leftClimberMotor.setInverted(false);
    m_leftClimberMotor.setIdleMode(IdleMode.kBrake);
    // m_leftClimberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // m_leftClimberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, _lClimberMaxHeight);

    m_leftClimberEncoder.setPositionConversionFactor(
        ClimberConstants.kClimberMetersPerRotation / ClimberConstants.kClimberGearRatio);
    m_LTargetSetpoint = m_leftClimberEncoder.getPosition();

    // set PID coefficients
    m_leftClimberController.setP(ClimberConstants.proportialPIDConstant);
    m_leftClimberController.setI(ClimberConstants.integralPIDConstant);
    m_leftClimberController.setD(ClimberConstants.derivativePIDConstant);
    m_leftClimberController.setIZone(ClimberConstants.integralPIDConstant);
    m_leftClimberController.setFF(ClimberConstants.leftFeedForwardPIDConstant);
    m_leftClimberController.setOutputRange(
        ClimberConstants.minPIDOutput, ClimberConstants.maxPIDOutput);

    m_rightClimberMotor.restoreFactoryDefaults();
    m_rightClimberEncoder = m_rightClimberMotor.getEncoder();
    m_rightClimberController = m_rightClimberMotor.getPIDController();
    m_rightClimberLimit =
        m_rightClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_rightClimberMotor.setInverted(false);
    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
    // m_rightClimberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // m_rightClimberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
    // _rClimberMaxHeight);

    m_rightClimberEncoder.setPositionConversionFactor(
        ClimberConstants.kClimberMetersPerRotation / ClimberConstants.kClimberGearRatio);
    m_RTargetSetpoint = m_rightClimberEncoder.getPosition();

    // set PID coefficients
    m_rightClimberController.setP(ClimberConstants.proportialPIDConstant);
    m_rightClimberController.setI(ClimberConstants.integralPIDConstant);
    m_rightClimberController.setD(ClimberConstants.derivativePIDConstant);
    m_rightClimberController.setIZone(ClimberConstants.integralPIDConstant);
    m_rightClimberController.setFF(ClimberConstants.leftFeedForwardPIDConstant);
    m_rightClimberController.setOutputRange(
        ClimberConstants.minPIDOutput, ClimberConstants.maxPIDOutput);

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
    if (m_isLReset && m_leftClimberLimit.isPressed()) {
      m_leftClimberEncoder.setPosition(0);
      m_leftClimberMotor.set(0.0);
      m_LTargetSetpoint = 0.0;
      m_isLReset = false;
      Shuffleboard.addEventMarker("isResetLClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetLClimber - done");
    }
    if (m_isRReset && m_rightClimberLimit.isPressed()) {
      m_rightClimberEncoder.setPosition(0);
      m_rightClimberMotor.set(0.0);
      m_RTargetSetpoint = 0.0;
      m_isRReset = false;
      Shuffleboard.addEventMarker("isResetRClimber - done: ", EventImportance.kHigh);
      System.out.println("isResetRClimber - done");
    }
    if (!isResetting()) {
      double _lextendPos = getLClimberPosition();
      double _rextendPos = getRClimberPosition();

      boolean extendLDone =
          Math.abs(_lextendPos - m_LTargetSetpoint) <= ClimberConstants.MoveThreshold;
      boolean extendRDone =
          Math.abs(_rextendPos - m_RTargetSetpoint) <= ClimberConstants.MoveThreshold;

      if (!extendLDone) {
        m_leftClimberController.setReference(m_LTargetSetpoint, CANSparkMax.ControlType.kPosition);
        if (DEBUGGING) {
          System.out.println("leftClimber - pos:" + _lextendPos + " setPt:" + m_LTargetSetpoint);
        }
      }
      if (!extendRDone) {
        m_rightClimberController.setReference(m_RTargetSetpoint, CANSparkMax.ControlType.kPosition);
        if (DEBUGGING) {
          System.out.println("rightClimber - pos:" + _rextendPos + " setPt:" + m_RTargetSetpoint);
        }
      }
    }
  }

  public void logPeriodic() {
    SmartDashboard.putBoolean("isResettingClimber", isResetting());
    SmartDashboard.putBoolean("atTargetHeight", atTargetHeight());
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void setClimberPosition(double lHeight, double rHeight) {
    if (!isResetting()) {
      m_LTargetSetpoint =
          MathUtil.clamp(lHeight, -0.025, ClimberConstants.kLClimberMaxHeight + 0.025);
      m_RTargetSetpoint =
          MathUtil.clamp(rHeight, -0.025, ClimberConstants.kRClimberMaxHeight + 0.025);
    }
  }

  public void setClimberPct(double pctLength) {
    this.setClimberPosition(
        MathUtil.interpolate(0, ClimberConstants.kLClimberMaxHeight, pctLength),
        MathUtil.interpolate(0, ClimberConstants.kRClimberMaxHeight, pctLength));
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

  public boolean atTargetHeight() {
    return (Math.abs(getLClimberPosition() - m_LTargetSetpoint) < ClimberConstants.MoveThreshold)
        && (Math.abs(getRClimberPosition() - m_RTargetSetpoint) < ClimberConstants.MoveThreshold);
  }

  public boolean isLClimbMinLimitSwitch() {
    return m_leftClimberLimit.isPressed();
  }

  public boolean isRClimbMinLimitSwitch() {
    return m_rightClimberLimit.isPressed();
  }

  public void teleop(double lval, double rval) {
    if (!isResetting()) {
      if ((lval != 0.0) && (rval != 0.0)) {
        this.setClimberPosition(m_LTargetSetpoint + lval, m_RTargetSetpoint + rval);
      }
    }
  }
}
