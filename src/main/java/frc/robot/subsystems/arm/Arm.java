package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import java.util.List;

import frc.robot.Robot;
import frc.robot.RobotPreferences;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** */
public class Arm extends SubsystemBase {
  public PneumaticHub m_pH = new PneumaticHub(PNEUMATICSHUB_ID);
  public DoubleSolenoid m_gripper = m_pH.makeDoubleSolenoid(SOLENOID_FWD_CHANNEL, SOLENOID_REV_CHANNEL);

  public CANSparkMax m_armLiftMotor = new CANSparkMax(ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_armLiftController;
  private RelativeEncoder m_armLiftEncoder;
  private SparkMaxLimitSwitch m_armLiftLimit;

  public CANSparkMax m_armExtendMotor = new CANSparkMax(ARM_EXTEND_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_armExtendController;
  private RelativeEncoder m_armExtendEncoder;
  private SparkMaxLimitSwitch m_armExtendLimit;

  private double m_armLiftSetpoint = 0;
  private double m_armLiftSetpointZero = 0;
  private double m_armExtendSetpoint = 0;
  private double m_armExtendSetpointZero = 0;
  private boolean m_Resetting = false;
  private boolean m_TeleopMode = false;
  private List<Pair<Double, Double>> liftProfile = List.of(
    new Pair<>(5.0, 20.0),
    new Pair<>(10.0, 80.0),
    new Pair<>(20.0, 150.0),
    new Pair<>(25.0, 300.0)
  );
  
  public Arm() {
    m_armLiftMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armLiftController = m_armLiftMotor.getPIDController();
    m_armLiftEncoder = m_armLiftMotor.getEncoder();
    m_armLiftLimit = m_armLiftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

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
    m_armLiftSetpointZero = 0;

    m_armExtendMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armExtendController = m_armExtendMotor.getPIDController();
    m_armExtendEncoder = m_armExtendMotor.getEncoder();
    m_armExtendLimit = m_armExtendMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

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
    m_armExtendSetpointZero = 0;

    initLogging();
  }

  @Override
  public void periodic() {
    if (m_Resetting) {
      if (m_armExtendLimit.isPressed()) {
        m_armExtendSetpointZero = m_armExtendEncoder.getPosition();
        m_armExtendMotor.set(0.0);
        m_armLiftMotor.set(RobotPreferences.ArmLift.armMinOutput.get());
      }
      if (m_armLiftLimit.isPressed()) {
        m_armLiftSetpointZero = m_armLiftEncoder.getPosition();
        m_armLiftMotor.set(0.0);
        m_Resetting = false;
      }
    } else {
      double ht = this.getArmLiftPosition();
      
      Pair<Double, Double> lastPair = new Pair<Double,Double>(0.0, 0.0);
      for (Pair<Double, Double> pair : liftProfile) {
        if (ht <= pair.getFirst()) {
          Double htPct = (ht - lastPair.getFirst()) / (pair.getFirst() - lastPair.getFirst());
          Double newLn = Math.min(MathUtil.interpolate(lastPair.getSecond(), pair.getSecond(), htPct), m_armExtendSetpoint);

          if (Math.abs(newLn - this.getArmExtendPosition()) > 0.01) {
            m_armLiftController.setReference(ht + m_armLiftSetpointZero, CANSparkMax.ControlType.kPosition);
            m_armExtendController.setReference(newLn + m_armExtendSetpointZero, CANSparkMax.ControlType.kPosition);
          } else if (Math.abs(ht - m_armLiftSetpoint) > 0.01) {
            m_armLiftController.setReference(m_armLiftSetpoint + m_armLiftSetpointZero, CANSparkMax.ControlType.kPosition);
          }
          break;
        }
        lastPair = pair;
      }
    }
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_armLiftMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_armExtendMotor, DCMotor.getNEO(1));
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    REVPhysicsSim.getInstance().run();
  }

  public void resetArm() {
    m_Resetting = true;
    m_armExtendMotor.set(RobotPreferences.ArmExtend.armMinOutput.get());
  }

  public boolean isResetting() {
    return m_Resetting;
  }

  public void setArmLiftPosition(double position) {
    if (!m_Resetting) {
      m_armLiftSetpoint = MathUtil.clamp(position, RobotPreferences.ArmLift.armMinPosition.get(), RobotPreferences.ArmLift.armMaxPosition.get());
      if (Robot.isReal()) {
        m_armLiftController.setReference(m_armLiftSetpoint + m_armLiftSetpointZero, CANSparkMax.ControlType.kPosition);
      } else {
        m_armLiftMotor.set(m_armLiftSetpoint > this.getArmLiftPosition() ? 
          RobotPreferences.ArmLift.armMaxOutput.get() :
          RobotPreferences.ArmLift.armMinOutput.get());
      }
    }
  }

  public double getArmLiftPosition() {
    return m_armLiftEncoder.getPosition() - m_armLiftSetpointZero;
  }

  public void raiseArm(double pctHeight) {
    this.setArmLiftPosition(MathUtil.interpolate(RobotPreferences.ArmLift.armMinPosition.get(), RobotPreferences.ArmLift.armMaxPosition.get(), pctHeight));
  }

  public boolean isArmRaised() {
    return Math.abs(getArmLiftPosition() - m_armLiftSetpoint) < 0.01;
  }

  public void setArmExtendPosition(double position) {
    if (!m_Resetting) {
      m_armExtendSetpoint = MathUtil.clamp(position, RobotPreferences.ArmExtend.armMinPosition.get(), RobotPreferences.ArmExtend.armMaxPosition.get());
      if (Robot.isReal()) {
        m_armExtendController.setReference(m_armExtendSetpoint + m_armExtendSetpointZero, CANSparkMax.ControlType.kPosition);
      } else {
        m_armExtendMotor.set(m_armExtendSetpoint > this.getArmExtendPosition() ? 
          RobotPreferences.ArmExtend.armMaxOutput.get() :
          RobotPreferences.ArmExtend.armMinOutput.get());
      }

    }
  }

  public double getArmExtendPosition() {
    return m_armExtendEncoder.getPosition() - m_armExtendSetpointZero;
  }

  public void extendArm(double pctLength) {
    this.setArmExtendPosition(MathUtil.interpolate(RobotPreferences.ArmExtend.armMinPosition.get(), RobotPreferences.ArmExtend.armMaxPosition.get(), pctLength));
  }

  public boolean isArmExtended() {
    return Math.abs(getArmExtendPosition() - m_armExtendSetpoint) < 0.01;
  }

  public void gripClaw(boolean close) {
    m_gripper.set(close ? Value.kReverse : Value.kForward);
  }

  public void gripOpen() {
    this.gripClaw(false);
  }

  public void gripClose() {
    this.gripClaw(true);
  }

  public void gripToggle() {
    this.gripClaw(isGripClawOpen());
  }

  public void gripReset(boolean close) {
    m_gripper.set(Value.kOff);
  }

  public boolean isGripClawOpen() {
    return (m_gripper.get() != Value.kReverse);
  }

  public boolean isArmExtendMinLimitSwitch() {
    return m_armExtendLimit.isPressed();
  }

  public boolean isArmLiftMinLimitSwitch() {
      return m_armLiftLimit.isPressed();
  }

  public void teleop(double liftVal, double extendVal) {
    if ((liftVal > 0.0) || (extendVal > 0.0)) {
      m_TeleopMode = true;
    }
    if (m_TeleopMode) {
      this.raiseArm(liftVal);
      this.extendArm(extendVal);

      m_TeleopMode = !((liftVal == 0.0) && (extendVal == 0.0));
    }
  }

  public void setArmPosition0() {
    this.setArmLiftPosition(RobotPreferences.ArmLift.armPosition0.get());
    this.setArmExtendPosition(RobotPreferences.ArmExtend.armPosition0.get());
  }

  public void setArmPosition1() {
    this.setArmLiftPosition(RobotPreferences.ArmLift.armPosition1.get());
    this.setArmExtendPosition(RobotPreferences.ArmExtend.armPosition1.get());
  }

  public void setArmPosition2() {
    this.setArmLiftPosition(RobotPreferences.ArmLift.armPosition2.get());
    this.setArmExtendPosition(RobotPreferences.ArmExtend.armPosition2.get());
  }

  public void initLogging() {
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain.addNumber("ArmLift/Position", m_armLiftEncoder::getPosition);
    tabMain.addNumber("ArmLift/SetPoint",() -> m_armLiftSetpoint);
    tabMain.addNumber("ArmExtend/Position", m_armExtendEncoder::getPosition);
    tabMain.addNumber("ArmExtend/SetPoint",() -> m_armExtendSetpoint);

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab("ARM");
      tab.addNumber("ArmLift/Process Variable", m_armLiftEncoder::getPosition);
      tab.addNumber("ArmLift/Output", m_armLiftMotor::getAppliedOutput);
      tab.addNumber("ArmLift/SetPoint",() -> m_armLiftSetpoint);
      tab.addNumber("ArmLift/SetPoint Zero",() -> m_armLiftSetpointZero);
      tab.addBoolean("ArmLift/Reverse LimitSwitch",this::isArmLiftMinLimitSwitch);

      tab.addNumber("ArmExtend/Process Variable", m_armExtendEncoder::getPosition);
      tab.addNumber("ArmExtend/Lift Output", m_armExtendMotor::getAppliedOutput);
      tab.addNumber("ArmExtend/SetPoint",() -> m_armExtendSetpoint);
      tab.addNumber("ArmExtend/SetPoint Zero",() -> m_armExtendSetpointZero);
      tab.addBoolean("ArmExtend/Reverse LimitSwitch",this::isArmExtendMinLimitSwitch);
    }
  }
}
