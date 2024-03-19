package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor = new TalonFX(WristConstants.WRISTMOTOR_ID, "CANivore");
  private final CANcoder m_canCoder = new CANcoder(WristConstants.CANCODER_ID, "CANivore");

  // private final PositionDutyCycle m_wristrequest = new PositionDutyCycle(0).withSlot(0);
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

  private double wristTargetPosition = 0;
  private boolean m_isTeleop = true;

  public WristSubsystem() {

    TalonFXConfiguration configWrist = new TalonFXConfiguration();

    configWrist.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configWrist.Slot0.kS = WristConstants.wristMotorKS;
    configWrist.Slot0.kV = WristConstants.wristMotorKV;
    configWrist.Slot0.kA = WristConstants.wristMotorKA;
    configWrist.Slot0.kP = WristConstants.wristMotorKP;
    configWrist.Slot0.kI = WristConstants.wristMotorKI;
    configWrist.Slot0.kD = WristConstants.wristMotorKD;

    MotionMagicConfigs configMagic = configWrist.MotionMagic;
    configMagic.MotionMagicCruiseVelocity = WristConstants.MMagicCruiseVelocity;
    configMagic.MotionMagicAcceleration = WristConstants.MMagicAcceleration;
    configMagic.MotionMagicJerk = WristConstants.MMagicJerk;

    m_wristMotor.getConfigurator().apply(configWrist);
    m_wristMotor.setPosition(
        m_canCoder.getAbsolutePosition().getValue() * WristConstants.kWristGearRatio);
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop
    // if (this.getPosition() <= 0.01) {
    //   m_wristMotor.stopMotor();
    // }
    updateSmartDashboard();
  }

  public void up() {
    this.setPosition(WristConstants.kTargetWristHigh);
  }

  public void down() {
    this.setPosition(WristConstants.kTargetWristLow);
  }

  public void setPosition(double pos) {
    m_isTeleop = false;
    wristTargetPosition = pos;

    // m_wristMotor.setControl(m_wristrequest.withPosition(wristTargetPosition));
    m_wristMotor.setControl(m_request.withPosition(wristTargetPosition));
  }

  public double getPosition() {
    return m_wristMotor.getPosition().getValue();
  }

  public boolean getOnTarget() {
    return Math.abs(this.getPosition() - wristTargetPosition) < 0.1;
  }

  public void setSpeed(double wspeed) {
    wristTargetPosition = 0;
    m_isTeleop = true;
    m_wristMotor.set(wspeed);
  }

  public void stop() {
    this.setSpeed(0.0);
  }

  public void teleop(double val) {
    val = MathUtil.applyDeadband(val, STICK_DEADBAND) * 0.1;

    if (m_isTeleop || (val != 0.0)) {
      this.setSpeed(val);
    }
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Wrist Postion", this.getPosition());
    SmartDashboard.putNumber("WristCanCoder Postion", m_canCoder.getPosition().getValue());
    SmartDashboard.putNumber("Wrist TargetPostion", wristTargetPosition);
  }
}
