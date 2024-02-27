package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor = new TalonFX(WristConstants.WRISTMOTOR_ID);
  private final CANcoder m_canCoder = new CANcoder(WristConstants.CANCODER_ID);

  private final PositionDutyCycle m_wristrequest = new PositionDutyCycle(0).withSlot(0);
  // private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);

  private double wristTargetPosition = 0;

  public WristSubsystem() {

    TalonFXConfiguration configWrist = new TalonFXConfiguration();
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
    m_wristMotor.setPosition(m_canCoder.getPosition().getValue());
  }

  public void up() {
    this.setPosition(WristConstants.kTargetWristHigh);
  }

  public void down() {
    this.setPosition(WristConstants.kTargetWristLow);
  }

  public void setPosition(double pos) {
    wristTargetPosition = pos;

    m_wristMotor.setControl(m_wristrequest.withPosition(wristTargetPosition));
    // m_wristMotor.setControl(m_request.withPosition(wristTargetPosition));
  }

  public double getPosition() {
    return m_canCoder.getPosition().getValue();
  }

  public boolean getOnTarget() {
    return Math.abs(this.getPosition() - wristTargetPosition) < 0.1;
  }

  public void setSpeed(double wspeed) {
    wristTargetPosition = 0;
    m_wristMotor.set(wspeed);
  }

  public void stop() {
    this.setSpeed(0.0);
  }
}
