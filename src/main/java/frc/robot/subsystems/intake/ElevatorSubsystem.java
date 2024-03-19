// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Change to elevator for shooter with NEOs

package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
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
      new SimableCANSparkMax(ElevatorConstants.LEFTELEVATOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_leftElevatorEncoder;
  private final SparkPIDController m_leftElevatorController;

  private final SimableCANSparkMax m_rightElevatorMotor =
      new SimableCANSparkMax(ElevatorConstants.RIGHTELEVATOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_rightElevatorEncoder;
  private final SparkPIDController m_rightElevatorController;

  private final CANcoder m_canCoder = new CANcoder(ElevatorConstants.CANCODER_ID);

  private double m_targetHeight = 0.0;
  private boolean m_isTeleop = true;

  private ISimWrapper mLeftSim;
  private ISimWrapper mRightSim;

  public ElevatorSubsystem() {

    m_leftElevatorEncoder = m_leftElevatorMotor.getEncoder();
    m_leftElevatorController = m_leftElevatorMotor.getPIDController();

    m_rightElevatorEncoder = m_rightElevatorMotor.getEncoder();
    m_rightElevatorController = m_rightElevatorMotor.getPIDController();

    m_leftElevatorMotor.restoreFactoryDefaults();
    m_leftElevatorMotor.setInverted(false);
    m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_leftElevatorEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.kElevatorGearRatio);
    m_leftElevatorEncoder.setPosition(m_canCoder.getAbsolutePosition().getValue());

    m_leftElevatorController.setP(ElevatorConstants.proportialPIDConstant);
    m_leftElevatorController.setI(ElevatorConstants.integralPIDConstant);
    m_leftElevatorController.setD(ElevatorConstants.derivativePIDConstant);
    m_leftElevatorController.setIZone(ElevatorConstants.integralPIDZone);
    m_leftElevatorController.setFF(ElevatorConstants.feedForwardPIDConstant);
    m_leftElevatorController.setOutputRange(
        ElevatorConstants.minPIDOutput, ElevatorConstants.maxPIDOutput);
    m_leftElevatorMotor.burnFlash();

    m_rightElevatorMotor.restoreFactoryDefaults();
    m_rightElevatorMotor.setInverted(true);
    m_rightElevatorMotor.setIdleMode(IdleMode.kCoast);
    m_rightElevatorEncoder.setPositionConversionFactor(1.0 / ElevatorConstants.kElevatorGearRatio);
    m_rightElevatorEncoder.setPosition(m_canCoder.getAbsolutePosition().getValue());

    m_rightElevatorController.setP(ElevatorConstants.proportialPIDConstant);
    m_rightElevatorController.setI(ElevatorConstants.integralPIDConstant);
    m_rightElevatorController.setD(ElevatorConstants.derivativePIDConstant);
    m_rightElevatorController.setIZone(ElevatorConstants.integralPIDZone);
    m_rightElevatorController.setFF(ElevatorConstants.feedForwardPIDConstant);
    m_rightElevatorController.setOutputRange(
        ElevatorConstants.minPIDOutput, ElevatorConstants.maxPIDOutput);
    m_rightElevatorMotor.burnFlash();

    // m_rightElevatorMotor.follow(m_leftElevatorMotor);

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
    // if (this.getPosition() <= 0.01) {
    //   m_leftElevatorMotor.stopMotor();
    // }

    updateSmartDashboard();
  }

  public void up() {
    this.setHeight(ElevatorConstants.kTargetElevatorHigh);
  }

  public void down() {
    this.setHeight(ElevatorConstants.kTargetElevatorLow);
  }

  public void setHeight(double ht) {
    m_isTeleop = false;
    m_targetHeight = Math.min(ht, ElevatorConstants.kElevatorMaxHeight);
    m_leftElevatorController.setReference(m_targetHeight, CANSparkBase.ControlType.kPosition);
    m_rightElevatorController.setReference(m_targetHeight, CANSparkBase.ControlType.kPosition);
    // m_rightElevatorMotor.follow(m_leftElevatorMotor);
  }

  public void setSpeed(double speed) {
    m_isTeleop = true;
    m_targetHeight = 0.0;
    m_leftElevatorMotor.set(speed);
    m_rightElevatorMotor.set(speed);
  }

  public void stop() {
    this.setSpeed(0);
  }

  public double getPosition() {
    return m_leftElevatorEncoder.getPosition();
  }

  public boolean getOnTarget() {
    return Math.abs(this.getPosition() - m_targetHeight) < ElevatorConstants.kOnTargetThreshold;
  }

  public void teleop(double val) {
    val = MathUtil.applyDeadband(val, STICK_DEADBAND) * 0.5;

    if (m_isTeleop || (val != 0.0)) {
      this.setSpeed(val);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Elevator Postion", this.getPosition());
    SmartDashboard.putNumber(
        "ElevatorCanCoder Postion", m_canCoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Elevator TargetPostion", m_targetHeight);
    // SmartDashboard.putNumber("RClimber Postion", m_rightElevatorEncoder.getPosition());
  }
}
