package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;

/** */
public class Arm extends SubsystemBase {
  PneumaticHub m_pH = new PneumaticHub(PNEUMATICSHUB_ID);
  DoubleSolenoid m_gripper = m_pH.makeDoubleSolenoid(SOLENOID_FWD_CHANNEL, SOLENOID_REV_CHANNEL);

// private Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
// private DoubleSolenoid m_gripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);

  private CANSparkMax m_armLiftMotor = new CANSparkMax(ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_armLiftController;
  private RelativeEncoder m_armLiftEncoder;

  private CANSparkMax m_armExtendMotor = new CANSparkMax(ARM_EXTEND_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_armExtendController;
  private RelativeEncoder m_armExtendEncoder;

  private double m_armLiftSetpoint = 0;
  private double m_armExtendSetpoint = 0;
  
  public Arm() {

    m_armLiftMotor.restoreFactoryDefaults();

    m_armLiftController = m_armLiftMotor.getPIDController();
    m_armLiftEncoder = m_armLiftMotor.getEncoder();

    // set PID coefficients
    m_armLiftController.setP(RobotPreferences.ArmLift.armKP.get());
    m_armLiftController.setI(RobotPreferences.ArmLift.armKI.get());
    m_armLiftController.setD(RobotPreferences.ArmLift.armKD.get());
    m_armLiftController.setIZone(RobotPreferences.ArmLift.armKIZ.get());
    m_armLiftController.setFF(RobotPreferences.ArmLift.armKFF.get());
    m_armLiftController.setOutputRange(RobotPreferences.ArmLift.armMinOutput.get(), RobotPreferences.ArmLift.armMaxOutput.get());

    // int smartMotionSlot = 0;
    // m_armLiftController.setSmartMotionMaxVelocity(RobotPreferences.ArmLift.armMaxVel.get(), smartMotionSlot);
    // m_armLiftController.setSmartMotionMinOutputVelocity(RobotPreferences.ArmLift.armMinVel.get(), smartMotionSlot);
    // m_armLiftController.setSmartMotionMaxAccel(RobotPreferences.ArmLift.armMaxAcc.get(), smartMotionSlot);
    // m_armLiftController.setSmartMotionAllowedClosedLoopError(RobotPreferences.ArmLift.armAllowedErr.get(), smartMotionSlot);

    // m_armLiftEncoder.setPositionConversionFactor(RobotPreferences.ArmLift.armGearRatio.get() * RobotPreferences.ArmLift.armMetersPerRotation.get());
    m_armLiftSetpoint = m_armLiftEncoder.getPosition();

    m_armExtendMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armExtendController = m_armExtendMotor.getPIDController();
    m_armExtendEncoder = m_armExtendMotor.getEncoder();

    // set PID coefficients
    m_armExtendController.setP(RobotPreferences.ArmExtend.armKP.get());
    m_armExtendController.setI(RobotPreferences.ArmExtend.armKI.get());
    m_armExtendController.setD(RobotPreferences.ArmExtend.armKD.get());
    m_armExtendController.setIZone(RobotPreferences.ArmExtend.armKIZ.get());
    m_armExtendController.setFF(RobotPreferences.ArmExtend.armKFF.get());
    m_armExtendController.setOutputRange(RobotPreferences.ArmExtend.armMinOutput.get(), RobotPreferences.ArmExtend.armMaxOutput.get());

    // m_armExtendController.setSmartMotionMaxVelocity(RobotPreferences.Arm.armMaxVel.get(), smartMotionSlot);
    // m_armExtendController.setSmartMotionMinOutputVelocity(RobotPreferences.Arm.armMinVel.get(), smartMotionSlot);
    // m_armExtendController.setSmartMotionMaxAccel(RobotPreferences.Arm.armMaxAcc.get(), smartMotionSlot);
    // m_armExtendController.setSmartMotionAllowedClosedLoopError(RobotPreferences.Arm.armAllowedErr.get(), smartMotionSlot);

    // m_armExtendEncoder.setPositionConversionFactor(RobotPreferences.Arm.armExtendGearRatio.get() * RobotPreferences.Arm.armExtendMetersPerRotation.get());
    m_armExtendSetpoint = m_armExtendEncoder.getPosition();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  public void logPeriodic() {
    SmartDashboard.putNumber("Lift Process Variable", m_armLiftEncoder.getPosition());
    SmartDashboard.putNumber("Extend Process Variable", m_armExtendEncoder.getPosition());
    SmartDashboard.putNumber("Lift Output", m_armLiftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Extend Output", m_armExtendMotor.getAppliedOutput());
  }

  public void setArmLiftPosition(double position) {
    m_armLiftSetpoint = position;
    m_armLiftController.setReference(m_armLiftSetpoint, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("Arm Lift SetPoint", m_armLiftSetpoint);
  }

  public double getArmLiftPosition() {
    return m_armLiftEncoder.getPosition();
  }

  public void raiseArm(double pctHeight) {
    this.setArmLiftPosition(MathUtil.interpolate(RobotPreferences.ArmLift.armMinPosition.get(), RobotPreferences.ArmLift.armMaxPosition.get(), pctHeight));
  }

  public boolean isArmRaised() {
    return Math.abs(getArmLiftPosition() - m_armLiftSetpoint) < 0.01;
  }

  public void raiseArm() {
    raiseArm(m_armLiftSetpoint + 1.0);
  }

  public void lowerArm() {
    raiseArm(m_armLiftSetpoint - 1.0);
  }

  public void setArmExtendPosition(double position) {
    m_armExtendSetpoint = position;
    m_armExtendController.setReference(m_armExtendSetpoint, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("Arm Extend SetPoint", m_armExtendSetpoint);
  }

  public double getArmExtendPosition() {
    return m_armExtendEncoder.getPosition();
  }

  public void extendArm(double pctLength) {
    this.setArmExtendPosition(MathUtil.interpolate(RobotPreferences.ArmExtend.armMinPosition.get(), RobotPreferences.ArmExtend.armMaxPosition.get(), pctLength));
  }

  public boolean isArmExtended() {
    return Math.abs(getArmExtendPosition() - m_armExtendSetpoint) < 0.01;
  }

  public void extendArm() {
    raiseArm(m_armExtendSetpoint + 1.0);
  }

  public void retractArm() {
    raiseArm(m_armExtendSetpoint - 1.0);
  }

  public void gripClaw(boolean close) {
    m_gripper.set(close ? Value.kReverse : Value.kForward);
  }

  public void gripReset(boolean close) {
    m_gripper.set(Value.kOff);
  }

  public boolean isGripClawOpen() {
    return (m_gripper.get() != Value.kReverse);
  }
}
