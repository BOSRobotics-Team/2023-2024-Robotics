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
  private boolean m_targetCones = true;

  public Arm() {
    m_armLiftMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_armLiftController = m_armLiftMotor.getPIDController();
    m_armLiftEncoder = m_armLiftMotor.getEncoder();
    m_armLiftLimit = m_armLiftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // set PID coefficients
    m_armLiftController.setP(ArmConstants.armLiftKP);
    m_armLiftController.setI(ArmConstants.armLiftKI);
    m_armLiftController.setD(ArmConstants.armLiftKD);
    m_armLiftController.setIZone(ArmConstants.armLiftKIZ);
    m_armLiftController.setFF(ArmConstants.armLiftKFF);
    m_armLiftController.setOutputRange(
        ArmConstants.armLiftMinOutput, ArmConstants.armLiftMaxOutput);

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
    m_armExtendController.setP(ArmConstants.armExtendKP);
    m_armExtendController.setI(ArmConstants.armExtendKI);
    m_armExtendController.setD(ArmConstants.armExtendKD);
    m_armExtendController.setIZone(ArmConstants.armExtendKIZ);
    m_armExtendController.setFF(ArmConstants.armExtendKFF);
    m_armExtendController.setOutputRange(
        ArmConstants.armExtendMinOutput, ArmConstants.armExtendMaxOutput);

    // m_armExtendEncoder.setPositionConversionFactor(RobotPreferences.Arm.armExtendGearRatio.get()
    // * RobotPreferences.Arm.armExtendMetersPerRotation.get());
    m_armExtendSetpoint = m_armExtendEncoder.getPosition();

    if (RobotBase.isSimulation()) {
      mLiftSim =
          new SingleJointedArmSimWrapper(
              new SingleJointedArmSim(
                  DCMotor.getNEO(1),
                  ArmConstants.armLiftGearRatio,
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
                  DCMotor.getNEO(1),
                  ArmConstants.armExtendGearRatio,
                  3.0,
                  Units.inchesToMeters(1.0),
                  0.0,
                  550.0,
                  false),
              new RevMotorControllerSimWrapper(m_armExtendMotor),
              RevEncoderSimWrapper.create(m_armExtendMotor));
    }

    initLiftProfile();
    initLogging();
  }

  void initLiftProfile() {
    liftProfile.clear();

    liftProfile.add(new Pair<Double, Double>(0.0, 0.0));
    for (double[] pairs : ArmConstants.armLiftProfile) {
      liftProfile.add(new Pair<Double, Double>(pairs[0], pairs[1]));
    }
    liftProfile.add(
        new Pair<Double, Double>(
            ArmConstants.armLiftMaxPosition, ArmConstants.armExtendMaxPosition));

    // String liftProfileStr = RobotPreferences.ArmLift.liftProfileStr();
    // if (!liftProfileStr.isEmpty()) {
    //   liftProfile.clear();
    //   for (String pair : liftProfileStr.split(",")) {
    //     String[] items = pair.split(":");
    //     if (items.length > 1) {
    //       liftProfile.add(
    //           new Pair<Double, Double>(Double.parseDouble(items[0]),
    // Double.parseDouble(items[1])));
    //     }
    //   }
    // }
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
      double newLiftSetPoint = m_armLiftSetpoint;

      boolean goUp = (newLiftSetPoint - liftPos) > ArmConstants.armLiftMoveThreshold;
      boolean goDn = (newLiftSetPoint - liftPos) < -ArmConstants.armLiftMoveThreshold;

      Pair<Double, Double> lastPair = new Pair<Double, Double>(0.0, 0.0);
      for (Pair<Double, Double> pair : liftProfile) {
        if ((lastPair.getFirst() <= liftPos) && (liftPos <= pair.getFirst())) {
          newExtendSetpoint =
              Math.min(newExtendSetpoint, goDn ? lastPair.getSecond() : pair.getSecond());
        }
        if ((lastPair.getSecond() <= extendPos) && (extendPos <= pair.getSecond())) {
          if (goDn) {
            newLiftSetPoint = Math.min(liftPos, Math.max(newLiftSetPoint, lastPair.getFirst()));
          }
          break;
        }
        lastPair = pair;
      }
      boolean inSafeZoneHt = (liftPos >= ArmConstants.armLiftExtendSafetyHeight);
      if (goUp && !inSafeZoneHt) {
        newExtendSetpoint = Math.min(newExtendSetpoint, extendPos);
      }

      // double htPct = (liftPos - lastPair.getFirst()) / (pair.getFirst() - lastPair.getFirst());
      // newExtendSetpoint =
      //     Math.floor(
      //         Math.min(
      //             MathUtil.interpolate(lastPair.getSecond(), pair.getSecond(), htPct),
      //             m_armExtendSetpoint));

      // if (this.isGripClawOpen()) {
      //   newLiftSetPoint = Math.max(newLiftSetPoint, ArmConstants.armLiftClawSafetyHeight);
      // }

      // boolean inSafeZoneHt = true; //(liftPos >= ArmConstants.armLiftExtendSafetyHeight); //
      // 104.0);
      // boolean inSafeZoneLn = inSafeZoneHt || (extendPos <= ArmConstants.armExtendSafetyLength);
      // //  180.0);

      boolean liftDone = Math.abs(liftPos - newLiftSetPoint) <= ArmConstants.armLiftMoveThreshold;
      if (!liftDone) {
        m_armLiftController.setReference(newLiftSetPoint, CANSparkMax.ControlType.kPosition);
        if (DEBUGGING) {
          System.out.println(
              "armLift - pos:"
                  + liftPos
                  + " setPt:"
                  + m_armLiftSetpoint
                  + " newPt:"
                  + newLiftSetPoint);
        }
      }
      boolean extendDone =
          Math.abs(extendPos - newExtendSetpoint) <= ArmConstants.armExtendMoveThreshold;
      if (!extendDone) {
        m_armExtendController.setReference(newExtendSetpoint, CANSparkMax.ControlType.kPosition);
        if (DEBUGGING) {
          System.out.println(
              "armExtend - pos:"
                  + extendPos
                  + " setPt:"
                  + m_armExtendSetpoint
                  + " newPt:"
                  + newExtendSetpoint);
        }
      }

      // if (!liftDone && goUp) {
      //   m_armLiftController.setReference(newLiftSetPoint, CANSparkMax.ControlType.kPosition);
      //   m_armExtendController.setReference(
      //       inSafeZoneHt ? newExtendSetpoint : extendPos, CANSparkMax.ControlType.kPosition);
      //   System.out.println("goUp: ht:" + liftPos + " ln:" + extendPos + " lift:" +
      //   newLiftSetPoint + " ext:" + newExtendSetpoint);
      // } else if (!extendDone && !goUp) {
      //   m_armExtendController.setReference(newExtendSetpoint, CANSparkMax.ControlType.kPosition);
      //   m_armLiftController.setReference(
      //       inSafeZoneLn ? newLiftSetPoint : liftPos, CANSparkMax.ControlType.kPosition);
      //   System.out.println("extend: ht:" + liftPos + " ln:" + extendPos + " lift:" +
      //   newLiftSetPoint + " ext:" + newExtendSetpoint);
      // } else if (!liftDone || !extendDone) {
      //   m_armLiftController.setReference(newLiftSetPoint, CANSparkMax.ControlType.kPosition);
      //   m_armExtendController.setReference(newExtendSetpoint, CANSparkMax.ControlType.kPosition);
      //   System.out.println("liftDone: ht:" + liftPos + " ln:" + extendPos + " lift:" +
      //   newLiftSetPoint + " ext:" + newExtendSetpoint);
      // }
    }
  }

  public void doResetting() {
    if ((m_Resetting == 1) && m_armExtendLimit.isPressed()) {
      m_armExtendMotor.set(0.0);
      m_armExtendEncoder.setPosition(0.0);
      m_armExtendSetpoint = 0.0;

      m_Resetting = 2;
      m_armLiftMotor.set(ArmConstants.armLiftResetOutput);
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
    m_armExtendMotor.set(ArmConstants.armExtendResetOutput);
  }

  public boolean isResetting() {
    return m_Resetting > 0;
  }

  public void setArmLiftPosition(double position) {
    if (!isResetting()) {
      m_armLiftSetpoint =
          MathUtil.clamp(
              position, ArmConstants.armLiftMinPosition, ArmConstants.armLiftMaxPosition);
    }
  }

  public double getArmLiftPosition() {
    return m_armLiftEncoder.getPosition();
  }

  public void raiseArm(double pctHeight) {
    this.setArmLiftPosition(
        MathUtil.interpolate(
            ArmConstants.armLiftMinPosition, ArmConstants.armLiftMaxPosition, pctHeight));
  }

  public boolean isArmRaised() {
    return Math.abs(getArmLiftPosition() - m_armLiftSetpoint) < ArmConstants.armLiftMoveThreshold;
  }

  public void setArmExtendPosition(double position) {
    if (!isResetting()) {
      m_armExtendSetpoint =
          MathUtil.clamp(
              position, ArmConstants.armExtendMinPosition, ArmConstants.armExtendMaxPosition);
    }
  }

  public double getArmExtendPosition() {
    return m_armExtendEncoder.getPosition();
  }

  public void extendArm(double pctLength) {
    this.setArmExtendPosition(
        MathUtil.interpolate(
            ArmConstants.armExtendMinPosition, ArmConstants.armExtendMaxPosition, pctLength));
  }

  public boolean isArmExtended() {
    return Math.abs(getArmExtendPosition() - m_armExtendSetpoint)
        < ArmConstants.armExtendMoveThreshold;
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
    return (m_gripper.get() == Value.kForward);
  }

  public boolean isArmExtendMinLimitSwitch() {
    return m_armExtendLimit.isPressed();
  }

  public boolean isArmLiftMinLimitSwitch() {
    return m_armLiftLimit.isPressed();
  }

  public boolean isTargetCone() {
    return m_targetCones;
  }

  public void targetCones(boolean enable) {
    m_targetCones = enable;
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

  public void setArmPosition(int pos) {
    if ((pos >= 0) && (pos < ArmConstants.armPositionCone.length)) {
      double[] position =
          m_targetCones ? ArmConstants.armPositionCone[pos] : ArmConstants.armPositionCube[pos];
      this.setArmLiftPosition(position[0]);
      this.setArmExtendPosition(position[1]);
    }
  }

  public void initLogging() {
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    ShuffleboardLayout armLay = tabMain.getLayout("Arm", BuiltInLayouts.kList).withSize(4, 4);
    armLay.addNumber("Lift Position", m_armLiftEncoder::getPosition);
    armLay.addNumber("Lift SetPoint", () -> m_armLiftSetpoint);
    armLay.addBoolean("Lift LimitSwitch", this::isArmLiftMinLimitSwitch);
    armLay.addNumber("Extend Position", m_armExtendEncoder::getPosition);
    armLay.addNumber("Extend SetPoint", () -> m_armExtendSetpoint);
    armLay.addBoolean("Extend LimitSwitch", this::isArmExtendMinLimitSwitch);
    armLay.addBoolean("Target Cones", () -> m_targetCones);

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
