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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import java.util.ArrayList;
import java.util.List;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

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
  private int m_Resetting = 0; // 1 == resetting Extend, 2 == resetting Lift
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
          new SingleJointedArmSimWrapper(
              new SingleJointedArmSim(
                  DCMotor.getNEO(1),
                  192.0,
                  SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30.0), 5.0),
                  Units.inchesToMeters(30.0),
                  0.0,
                  60.0,
                  true),
              new RevMotorControllerSimWrapper(m_armLiftMotor),
              RevEncoderSimWrapper.create(m_armLiftMotor));
      mExtendSim =
          new ElevatorSimWrapper(
              new ElevatorSim(
                  DCMotor.getNEO(1), 80.0, 3.0, Units.inchesToMeters(1.0), 0.0, 550.0, false),
              new RevMotorControllerSimWrapper(m_armExtendMotor),
              RevEncoderSimWrapper.create(m_armExtendMotor));
    }

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
  public void simulationPeriodic() {
    mLiftSim.update();
    mExtendSim.update();
  }

  @Override
  public void periodic() {
    if (isResetting()) {
      doResetting();
    } else {
      double liftPos = this.getArmLiftPosition();
      double extendPos = this.getArmExtendPosition();
      double newExtendSetpoint = m_armExtendSetpoint;

      Pair<Double, Double> lastPair = new Pair<Double, Double>(0.0, 0.0);
      for (Pair<Double, Double> pair : liftProfile) {
        if (liftPos <= pair.getFirst()) {
          double htPct = (liftPos - lastPair.getFirst()) / (pair.getFirst() - lastPair.getFirst());
          newExtendSetpoint =
              Math.floor(
                  Math.min(
                      MathUtil.interpolate(lastPair.getSecond(), pair.getSecond(), htPct),
                      m_armExtendSetpoint));
          break;
        }
        lastPair = pair;
      }
      boolean liftDone = Math.abs(liftPos - m_armLiftSetpoint) <= 1.0;
      boolean extendDone = Math.abs(extendPos - newExtendSetpoint) <= 1.0;
      boolean goUp = (m_armLiftSetpoint - liftPos) > 1.0;
      boolean inSafeZoneHt = (liftPos >= 26.0);
      boolean inSafeZoneLn = (extendPos < 180.0);

      if (!liftDone && goUp) {
        m_armLiftController.setReference(m_armLiftSetpoint, CANSparkMax.ControlType.kPosition);
        m_armExtendController.setReference(
            inSafeZoneHt ? newExtendSetpoint : extendPos, CANSparkMax.ControlType.kPosition);
        System.out.println(
            "goUp: ht:"
                + liftPos
                + " ln:"
                + extendPos
                + " lift:"
                + m_armLiftSetpoint
                + " ext:"
                + newExtendSetpoint);
      } else if (!extendDone && !goUp) {
        m_armExtendController.setReference(newExtendSetpoint, CANSparkMax.ControlType.kPosition);
        m_armLiftController.setReference(
            inSafeZoneLn ? m_armLiftSetpoint : liftPos, CANSparkMax.ControlType.kPosition);
        System.out.println(
            "extend: ht:"
                + liftPos
                + " ln:"
                + extendPos
                + " lift:"
                + m_armLiftSetpoint
                + " ext:"
                + newExtendSetpoint);
      } else if (!liftDone || !extendDone) {
        m_armLiftController.setReference(m_armLiftSetpoint, CANSparkMax.ControlType.kPosition);
        m_armExtendController.setReference(newExtendSetpoint, CANSparkMax.ControlType.kPosition);
        System.out.println(
            "liftDone: ht:"
                + liftPos
                + " ln:"
                + extendPos
                + " lift:"
                + m_armLiftSetpoint
                + " ext:"
                + newExtendSetpoint);
      }
    }
  }

  public void doResetting() {
    if ((m_Resetting == 1) && m_armExtendLimit.isPressed()) {
      m_armExtendMotor.set(0.0);
      m_armExtendEncoder.setPosition(0.0);
      m_armExtendSetpoint = 0.0;

      m_Resetting = 2;
      m_armLiftMotor.set(-0.1);
    }
    if ((m_Resetting == 2) && m_armLiftLimit.isPressed()) {
      m_armLiftEncoder.setPosition(0.0);
      m_armLiftMotor.set(0.0);
      m_armLiftSetpoint = 0.0;

      m_Resetting = 0;
    }
  }

  public void resetArm() {
    m_Resetting = 1;
    m_armExtendMotor.set(-0.2);
  }

  public boolean isResetting() {
    return m_Resetting > 0;
  }

  public void setArmLiftPosition(double position) {
    if (!isResetting()) {
      m_armLiftSetpoint =
          MathUtil.clamp(
              position,
              RobotPreferences.ArmLift.armMinPosition.get(),
              RobotPreferences.ArmLift.armMaxPosition.get());
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
    return Math.abs(getArmLiftPosition() - m_armLiftSetpoint) < 1.0;
  }

  public void setArmExtendPosition(double position) {
    if (!isResetting()) {
      m_armExtendSetpoint =
          MathUtil.clamp(
              position,
              RobotPreferences.ArmExtend.armMinPosition.get(),
              RobotPreferences.ArmExtend.armMaxPosition.get());
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
    return Math.abs(getArmExtendPosition() - m_armExtendSetpoint) < 1.0;
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
    if (!isResetting()) {
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

  public void setArmPosition3() {
    this.setArmLiftPosition(RobotPreferences.ArmLift.armPosition3.get());
    this.setArmExtendPosition(RobotPreferences.ArmExtend.armPosition3.get());
  }

  public void initLogging() {
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    ShuffleboardLayout armLay = tabMain.getLayout("Arm", BuiltInLayouts.kList).withSize(4, 4);
    armLay.addNumber("LiftPosition", m_armLiftEncoder::getPosition);
    armLay.addNumber("LiftSetPoint", () -> m_armLiftSetpoint);
    armLay.addNumber("ExtendPosition", m_armExtendEncoder::getPosition);
    armLay.addNumber("ExtendSetPoint", () -> m_armExtendSetpoint);

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab("ARM");

      ShuffleboardLayout liftLay =
          tab.getLayout("ArmLift", BuiltInLayouts.kList).withSize(4, 4).withPosition(0, 0);
      liftLay.addNumber("Position", m_armLiftEncoder::getPosition).withPosition(0, 0);
      liftLay.addNumber("Output", m_armLiftMotor::getAppliedOutput).withPosition(0, 1);
      liftLay.addNumber("SetPoint", () -> m_armLiftSetpoint).withPosition(0, 2);
      liftLay.addBoolean("Reverse LimitSwitch", this::isArmLiftMinLimitSwitch).withPosition(0, 3);

      ShuffleboardLayout extLay =
          tab.getLayout("ArmExtend", BuiltInLayouts.kList).withSize(4, 4).withPosition(4, 0);
      extLay.addNumber("Position", m_armExtendEncoder::getPosition).withPosition(0, 0);
      extLay.addNumber("Output", m_armExtendMotor::getAppliedOutput).withPosition(0, 1);
      extLay.addNumber("SetPoint", () -> m_armExtendSetpoint).withPosition(0, 2);
      extLay.addBoolean("Reverse LimitSwitch", this::isArmExtendMinLimitSwitch).withPosition(0, 3);
    }
  }
}
