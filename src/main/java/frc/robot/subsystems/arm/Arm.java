package frc.robot.subsystems.arm;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import java.util.ArrayList;
import java.util.List;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

/** */
public class Arm extends SubsystemBase {
  public final PneumaticHub m_pH = new PneumaticHub(PNEUMATICSHUB_ID);
  public final DoubleSolenoid m_gripper =
      m_pH.makeDoubleSolenoid(SOLENOID_FWD_CHANNEL, SOLENOID_REV_CHANNEL);

  public final SimableCANSparkMax m_armLiftMotor =
      new SimableCANSparkMax(ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxPIDController m_armLiftController;
  public final RelativeEncoder m_armLiftEncoder;
  public final SparkMaxLimitSwitch m_armLiftLimit;

  public final SimableCANSparkMax m_armExtendMotor =
      new SimableCANSparkMax(ARM_EXTEND_MOTOR_ID, MotorType.kBrushless);
  private final SparkMaxPIDController m_armExtendController;
  public final RelativeEncoder m_armExtendEncoder;
  public final SparkMaxLimitSwitch m_armExtendLimit;

  private ISimWrapper mLiftSim;
  private ISimWrapper mExtendSim;

  private double m_armLiftSetpoint = 0;
  private double m_armExtendSetpoint = 0;
  private boolean m_Resetting = false;
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

    if (RobotBase.isSimulation()) {
      mLiftSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNeo550(1), 1.0, 1.0, Units.inchesToMeters(2.0), 0.0, 80.0, false),
              new RevMotorControllerSimWrapper(m_armLiftMotor),
              RevEncoderSimWrapper.create(m_armLiftMotor));
      mExtendSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNeo550(1), 0.5, 1.0, Units.inchesToMeters(2.0), 0.0, 550.0, false),
              new RevMotorControllerSimWrapper(m_armExtendMotor),
              RevEncoderSimWrapper.create(m_armExtendMotor));
    }

    initLiftProfile();
    initLogging();
  }

  @Override
  public void simulationPeriodic() {
    mLiftSim.update();
    mExtendSim.update();
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
        m_armExtendEncoder.setPosition(0.0);
        m_armExtendMotor.set(0.0);
        m_armLiftMotor.set(RobotPreferences.ArmLift.armMinOutput.get());
      }
      if (m_armLiftLimit.isPressed()) {
        m_armLiftEncoder.setPosition(0.0);
        m_armLiftMotor.set(0.0);
        m_Resetting = false;
      }
    } else {
      double ht = this.getArmLiftPosition();
      double ln = this.getArmExtendPosition();
      double newLn = ln;

      Pair<Double, Double> lastPair = new Pair<Double, Double>(0.0, 0.0);
      for (Pair<Double, Double> pair : liftProfile) {
        if (ht <= pair.getFirst()) {
          double htPct = (ht - lastPair.getFirst()) / (pair.getFirst() - lastPair.getFirst());
          newLn =
              Math.min(
                  MathUtil.interpolate(lastPair.getSecond(), pair.getSecond(), htPct),
                  m_armExtendSetpoint);
          break;
        }
        lastPair = pair;
      }
      if (Math.abs(newLn - ln) > 0.5) {
        m_armLiftController.setReference(0.0, CANSparkMax.ControlType.kDutyCycle); // stop lift arm
        m_armExtendController.setReference(newLn, CANSparkMax.ControlType.kPosition);
        // System.out.println("LiftProfile: ht:" + ht + " ln:" + ln + " newLn:" + newLn);
      } else if (Math.abs(ht - m_armLiftSetpoint) > 0.5) {
        m_armLiftController.setReference(m_armLiftSetpoint, CANSparkMax.ControlType.kPosition);
        // System.out.println("LiftProfile: ht:" + ht + " ln:" + ln + " lift:" + m_armLiftSetpoint);
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
      m_armLiftController.setReference(m_armLiftSetpoint, CANSparkMax.ControlType.kPosition);
    }
  }

  public double getArmLiftPosition() {
    return m_armLiftEncoder.getPosition();
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
      m_armExtendController.setReference(m_armExtendSetpoint, CANSparkMax.ControlType.kPosition);
    }
  }

  public double getArmExtendPosition() {
    return m_armExtendEncoder.getPosition();
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
    if (!m_Resetting) {
      if (liftVal != 0.0) {
        this.setArmLiftPosition(m_armLiftSetpoint + liftVal);
      }
      if (extendVal != 0.0) {
        this.setArmExtendPosition(m_armExtendSetpoint + extendVal);
      }
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
      tab.addBoolean("ArmLift/Reverse LimitSwitch", this::isArmLiftMinLimitSwitch);

      tab.addNumber("ArmExtend/Position", m_armExtendEncoder::getPosition);
      tab.addNumber("ArmExtend/Lift Output", m_armExtendMotor::getAppliedOutput);
      tab.addNumber("ArmExtend/SetPoint", () -> m_armExtendSetpoint);
      tab.addBoolean("ArmExtend/Reverse LimitSwitch", this::isArmExtendMinLimitSwitch);
    }
  }
}
