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
  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid gripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);

  private CANSparkMax armLift = new CANSparkMax(Constants.ARM_LIFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax armExtend = new CANSparkMax(Constants.ARM_EXTEND_MOTOR, MotorType.kBrushless);

  private SparkMaxPIDController m_pidLiftController;
  private RelativeEncoder m_LiftEncoder;

  private SparkMaxPIDController m_pidExtendController;
  private RelativeEncoder m_ExtendEncoder;
  
  public Arm() {

    armLift.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_pidLiftController = armLift.getPIDController();
    m_LiftEncoder = armLift.getEncoder();

    // set PID coefficients
    m_pidLiftController.setP(Constants.ArmConstants.armKP);
    m_pidLiftController.setI(Constants.ArmConstants.armKI);
    m_pidLiftController.setD(Constants.ArmConstants.armKD);
    m_pidLiftController.setIZone(Constants.ArmConstants.armKIZ);
    m_pidLiftController.setFF(Constants.ArmConstants.armKFF);
    m_pidLiftController.setOutputRange(Constants.ArmConstants.armMinOutput, Constants.ArmConstants.armMaxOutput);

    int smartMotionSlot = 0;
    m_pidLiftController.setSmartMotionMaxVelocity(Constants.ArmConstants.armMaxVel, smartMotionSlot);
    m_pidLiftController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.armMinVel, smartMotionSlot);
    m_pidLiftController.setSmartMotionMaxAccel(Constants.ArmConstants.armMaxAcc, smartMotionSlot);
    m_pidLiftController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.armAllowedErr, smartMotionSlot);

    armExtend.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_pidExtendController = armExtend.getPIDController();
    m_ExtendEncoder = armExtend.getEncoder();

    // set PID coefficients
    m_pidExtendController.setP(Constants.ArmConstants.armKP);
    m_pidExtendController.setI(Constants.ArmConstants.armKI);
    m_pidExtendController.setD(Constants.ArmConstants.armKD);
    m_pidExtendController.setIZone(Constants.ArmConstants.armKIZ);
    m_pidExtendController.setFF(Constants.ArmConstants.armKFF);
    m_pidExtendController.setOutputRange(Constants.ArmConstants.armMinOutput, Constants.ArmConstants.armMaxOutput);

    m_pidExtendController.setSmartMotionMaxVelocity(Constants.ArmConstants.armMaxVel, smartMotionSlot);
    m_pidExtendController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.armMinVel, smartMotionSlot);
    m_pidExtendController.setSmartMotionMaxAccel(Constants.ArmConstants.armMaxAcc, smartMotionSlot);
    m_pidExtendController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.armAllowedErr, smartMotionSlot);

    addChild("Compressor", compressor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Process Variable", m_LiftEncoder.getPosition());
    SmartDashboard.putNumber("Extend Process Variable", m_ExtendEncoder.getPosition());
    SmartDashboard.putNumber("Lift Output", armLift.getAppliedOutput());
    SmartDashboard.putNumber("Extend Output", armExtend.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  public void raiseArm(double height) {
    double setPoint = height;
    m_pidLiftController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("Lift SetPoint", setPoint);

  }

  public double getArmHeight() {
    return m_LiftEncoder.getPosition();
  }

  public void extendArm(double length) {
    double setPoint = length;
    m_pidExtendController.setReference(length, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("Extend SetPoint", setPoint);
  }

  public double getArmExtension() {
    return m_ExtendEncoder.getPosition();
  }

  public void gripClaw(Boolean close) {
    gripper.set(close ? Value.kReverse : Value.kForward);
  }
}
