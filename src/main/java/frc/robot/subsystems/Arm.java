package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** */
public class Arm extends SubsystemBase {
  private Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid m_gripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);

  private CANSparkMax m_armLiftMotor = new CANSparkMax(Constants.ARM_LIFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_armExtendMotor = new CANSparkMax(Constants.ARM_EXTEND_MOTOR, MotorType.kBrushless);

  private SparkMaxPIDController m_armLiftController;
  private RelativeEncoder m_armLiftEncoder;

  private SparkMaxPIDController m_armExtendController;
  private RelativeEncoder m_armExtendEncoder;

  private double m_armLiftSetpoint = 0;
  private double m_armExtendSetpoint = 0;
  
  public Arm() {

    m_armLiftMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armLiftController = m_armLiftMotor.getPIDController();
    m_armLiftEncoder = m_armLiftMotor.getEncoder();

    // set PID coefficients
    m_armLiftController.setP(Constants.ArmConstants.armKP);
    m_armLiftController.setI(Constants.ArmConstants.armKI);
    m_armLiftController.setD(Constants.ArmConstants.armKD);
    m_armLiftController.setIZone(Constants.ArmConstants.armKIZ);
    m_armLiftController.setFF(Constants.ArmConstants.armKFF);
    m_armLiftController.setOutputRange(Constants.ArmConstants.armMinOutput, Constants.ArmConstants.armMaxOutput);

    int smartMotionSlot = 0;
    m_armLiftController.setSmartMotionMaxVelocity(Constants.ArmConstants.armMaxVel, smartMotionSlot);
    m_armLiftController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.armMinVel, smartMotionSlot);
    m_armLiftController.setSmartMotionMaxAccel(Constants.ArmConstants.armMaxAcc, smartMotionSlot);
    m_armLiftController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.armAllowedErr, smartMotionSlot);

    m_armLiftEncoder.setPositionConversionFactor(Constants.ArmConstants.armLiftGearRatio * Constants.ArmConstants.armLiftMetersPerRotation);
    m_armLiftSetpoint = m_armLiftEncoder.getPosition();

    m_armExtendMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armExtendController = m_armExtendMotor.getPIDController();
    m_armExtendEncoder = m_armExtendMotor.getEncoder();

    // set PID coefficients
    m_armExtendController.setP(Constants.ArmConstants.armKP);
    m_armExtendController.setI(Constants.ArmConstants.armKI);
    m_armExtendController.setD(Constants.ArmConstants.armKD);
    m_armExtendController.setIZone(Constants.ArmConstants.armKIZ);
    m_armExtendController.setFF(Constants.ArmConstants.armKFF);
    m_armExtendController.setOutputRange(Constants.ArmConstants.armMinOutput, Constants.ArmConstants.armMaxOutput);

    m_armExtendController.setSmartMotionMaxVelocity(Constants.ArmConstants.armMaxVel, smartMotionSlot);
    m_armExtendController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.armMinVel, smartMotionSlot);
    m_armExtendController.setSmartMotionMaxAccel(Constants.ArmConstants.armMaxAcc, smartMotionSlot);
    m_armExtendController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.armAllowedErr, smartMotionSlot);

    m_armExtendEncoder.setPositionConversionFactor(Constants.ArmConstants.armExtendGearRatio * Constants.ArmConstants.armExtendMetersPerRotation);
    m_armExtendSetpoint = m_armExtendEncoder.getPosition();

    addChild("Compressor", m_compressor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Process Variable", m_armLiftEncoder.getPosition());
    SmartDashboard.putNumber("Extend Process Variable", m_armExtendEncoder.getPosition());
    SmartDashboard.putNumber("Lift Output", m_armLiftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Extend Output", m_armExtendMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  public void raiseArm(double height) {
    m_armLiftSetpoint = Math.min(Math.max(height, Constants.ArmConstants.armLiftMaxHeight), Constants.ArmConstants.armLiftMinHeight);
    m_armLiftController.setReference(m_armLiftSetpoint, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("Arm Lift SetPoint", m_armLiftSetpoint);
  }

  public void raiseArm() {
    raiseArm(m_armLiftSetpoint + 0.1);
  }

  public void lowerArm() {
    raiseArm(m_armLiftSetpoint - 0.1);
  }

  public boolean isArmRaised() {
    return Math.abs(getArmHeight() - m_armLiftSetpoint) < 0.01;
  }

  public double getArmHeight() {
    return m_armLiftEncoder.getPosition();
  }

  public void extendArm(double length) {
    m_armExtendSetpoint = Math.min(Math.max(length, Constants.ArmConstants.armExtendMaxLength), Constants.ArmConstants.armExtendMinLength);
    m_armExtendController.setReference(length, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("Arm Extend SetPoint", m_armExtendSetpoint);
  }

  public void extendArm() {
    raiseArm(m_armExtendSetpoint + 0.1);
  }

  public void retractArm() {
    raiseArm(m_armExtendSetpoint - 0.1);
  }

  public boolean isArmExtended() {
    return Math.abs(getArmExtension() - m_armExtendSetpoint) < 0.01;
  }

  public double getArmExtension() {
    return m_armExtendEncoder.getPosition();
  }

  public void gripClaw(boolean close) {
    m_gripper.set(close ? Value.kReverse : Value.kForward);
  }

  public boolean isGripClawOpen() {
    return (m_gripper.get() != Value.kReverse);
  }
}
