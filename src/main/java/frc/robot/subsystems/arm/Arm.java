package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import java.util.ArrayList;
import java.util.List;

/** */
public class Arm extends SubsystemBase {
  public final PneumaticHub m_pH = new PneumaticHub(PNEUMATICSHUB_ID);
  public final DoubleSolenoid m_gripper =
      m_pH.makeDoubleSolenoid(SOLENOID_FWD_CHANNEL, SOLENOID_REV_CHANNEL);

  public final CANSparkMax m_armLiftMotor =
      new CANSparkMax(ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxPIDController m_armLiftController;
  private final RelativeEncoder m_armLiftEncoder;
  private final SparkMaxLimitSwitch m_armLiftLimit;

  public final CANSparkMax m_armExtendMotor =
      new CANSparkMax(ARM_EXTEND_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxPIDController m_armExtendController;
  private final RelativeEncoder m_armExtendEncoder;
  private final SparkMaxLimitSwitch m_armExtendLimit;

  private double m_armLiftSetpoint = 0;
  private double m_armLiftSetpointZero = 0;
  private double m_armExtendSetpoint = 0;
  private double m_armExtendSetpointZero = 0;
  private boolean m_Resetting = false;
  private boolean m_TeleopMode = false;
  private List<Pair<Double, Double>> liftProfile = new ArrayList<Pair<Double, Double>>();

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
    m_armLiftController.setOutputRange(
        RobotPreferences.ArmLift.armMinOutput.get(), RobotPreferences.ArmLift.armMaxOutput.get());

    // m_armLiftEncoder.setPositionConversionFactor(RobotPreferences.ArmLift.armGearRatio.get()
    // * RobotPreferences.ArmLift.armMetersPerRotation.get());
    m_armLiftSetpoint = m_armLiftEncoder.getPosition();
    m_armLiftSetpointZero = 0;

    m_armExtendMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armExtendController = m_armExtendMotor.getPIDController();
    m_armExtendEncoder = m_armExtendMotor.getEncoder();
    m_armExtendLimit =
        m_armExtendMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // set PID coefficients
    m_armExtendController.setP(RobotPreferences.ArmExtend.armKP.get());
    m_armExtendController.setI(RobotPreferences.ArmExtend.armKI.get());
    m_armExtendController.setD(RobotPreferences.ArmExtend.armKD.get());
    m_armExtendController.setIZone(RobotPreferences.ArmExtend.armKIZ.get());
    m_armExtendController.setFF(RobotPreferences.ArmExtend.armKFF.get());
    m_armExtendController.setOutputRange(
        RobotPreferences.ArmExtend.armMinOutput.get(),
        RobotPreferences.ArmExtend.armMaxOutput.get());

    // m_armExtendEncoder.setPositionConversionFactor(RobotPreferences.Arm.armExtendGearRatio.get()
    // * RobotPreferences.Arm.armExtendMetersPerRotation.get());
    m_armExtendSetpoint = m_armExtendEncoder.getPosition();
    m_armExtendSetpointZero = 0;

    initLiftProfile();
    initLogging();
  }

  void initLiftProfile() {
    String liftProfileStr = RobotPreferences.ArmLift.liftProfileStr();
    if (!liftProfileStr.isEmpty()) {
      liftProfile.clear();
      for (String pair : liftProfileStr.split(",")) {
        String[] items = pair.split(":");
        if (items.length > 1) {
          liftProfile.add(
              new Pair<Double, Double>(Double.parseDouble(items[0]), Double.parseDouble(items[1])));
        }
      }
    }
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

      Pair<Double, Double> lastPair = new Pair<Double, Double>(0.0, 0.0);
      for (Pair<Double, Double> pair : liftProfile) {
        if (ht <= pair.getFirst()) {
          Double htPct = (ht - lastPair.getFirst()) / (pair.getFirst() - lastPair.getFirst());
          Double newLn =
              Math.min(
                  MathUtil.interpolate(lastPair.getSecond(), pair.getSecond(), htPct),
                  m_armExtendSetpoint);

          if (Math.abs(newLn - this.getArmExtendPosition()) > 0.01) {
            m_armLiftController.setReference(
                ht + m_armLiftSetpointZero, CANSparkMax.ControlType.kPosition);
            m_armExtendController.setReference(
                newLn + m_armExtendSetpointZero, CANSparkMax.ControlType.kPosition);
          } else if (Math.abs(ht - m_armLiftSetpoint) > 0.01) {
            m_armLiftController.setReference(
                m_armLiftSetpoint + m_armLiftSetpointZero, CANSparkMax.ControlType.kPosition);
          }
          break;
        }
        lastPair = pair;
      }
    }
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
      m_armLiftSetpoint =
          MathUtil.clamp(
              position,
              RobotPreferences.ArmLift.armMinPosition.get(),
              RobotPreferences.ArmLift.armMaxPosition.get());
      m_armLiftController.setReference(
          m_armLiftSetpoint + m_armLiftSetpointZero, CANSparkMax.ControlType.kPosition);
    }
  }

  public double getArmLiftPosition() {
    return m_armLiftEncoder.getPosition() - m_armLiftSetpointZero;
  }

  public void raiseArm(double pctHeight) {
    this.setArmLiftPosition(
        MathUtil.interpolate(
            RobotPreferences.ArmLift.armMinPosition.get(),
            RobotPreferences.ArmLift.armMaxPosition.get(),
            pctHeight));
  }

  public boolean isArmRaised() {
    return Math.abs(getArmLiftPosition() - m_armLiftSetpoint) < 0.01;
  }

  public void setArmExtendPosition(double position) {
    if (!m_Resetting) {
      m_armExtendSetpoint =
          MathUtil.clamp(
              position,
              RobotPreferences.ArmExtend.armMinPosition.get(),
              RobotPreferences.ArmExtend.armMaxPosition.get());
      m_armExtendController.setReference(
          m_armExtendSetpoint + m_armExtendSetpointZero, CANSparkMax.ControlType.kPosition);
    }
  }

  public double getArmExtendPosition() {
    return m_armExtendEncoder.getPosition() - m_armExtendSetpointZero;
  }

  public void extendArm(double pctLength) {
    this.setArmExtendPosition(
        MathUtil.interpolate(
            RobotPreferences.ArmExtend.armMinPosition.get(),
            RobotPreferences.ArmExtend.armMaxPosition.get(),
            pctLength));
  }

  public boolean isArmExtended() {
    return Math.abs(getArmExtendPosition() - m_armExtendSetpoint) < 0.01;
  }

  public void gripOpen() {
    m_gripper.set(Value.kForward);
  }

  public void gripClose() {
    m_gripper.set(Value.kReverse);
  }

  public void gripToggle() {
    m_gripper.set(isGripClawOpen() ? Value.kReverse : Value.kForward);
  }

  public void gripReset() {
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
    if (m_TeleopMode && !m_Resetting) {
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
    tabMain.addNumber("Arm/LiftPosition", m_armLiftEncoder::getPosition);
    tabMain.addNumber("Arm/LiftSetPoint", () -> m_armLiftSetpoint);
    tabMain.addNumber("Arm/ExtendPosition", m_armExtendEncoder::getPosition);
    tabMain.addNumber("Arm/ExtendSetPoint", () -> m_armExtendSetpoint);

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab("ARM");
      tab.addNumber("ArmLift/Position", m_armLiftEncoder::getPosition);
      tab.addNumber("ArmLift/Output", m_armLiftMotor::getAppliedOutput);
      tab.addNumber("ArmLift/SetPoint", () -> m_armLiftSetpoint);
      tab.addNumber("ArmLift/SetPoint Zero", () -> m_armLiftSetpointZero);
      tab.addBoolean("ArmLift/Reverse LimitSwitch", this::isArmLiftMinLimitSwitch);

      tab.addNumber("ArmExtend/Position", m_armExtendEncoder::getPosition);
      tab.addNumber("ArmExtend/Lift Output", m_armExtendMotor::getAppliedOutput);
      tab.addNumber("ArmExtend/SetPoint", () -> m_armExtendSetpoint);
      tab.addNumber("ArmExtend/SetPoint Zero", () -> m_armExtendSetpointZero);
      tab.addBoolean("ArmExtend/Reverse LimitSwitch", this::isArmExtendMinLimitSwitch);
    }
  }
}
