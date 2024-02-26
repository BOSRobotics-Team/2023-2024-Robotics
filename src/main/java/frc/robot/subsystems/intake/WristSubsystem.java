package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor = new TalonFX(ShooterConstants.WRISTMOTOR_ID, "Rio");

  private final VelocityVoltage m_wristrequest;
  private TalonFXConfiguration configWrist = new TalonFXConfiguration();

  private double wristTargetVelocity = 0;

  private int rollingAvg = 0;

  public WristSubsystem() {

    configWrist.Slot0.kS = WristConstants.wristMotorKS;
    configWrist.Slot0.kV = WristConstants.wristMotorKV;
    configWrist.Slot0.kP = WristConstants.wristMotorKP;
    configWrist.Slot0.kI = WristConstants.wristMotorKI;
    configWrist.Slot0.kD = WristConstants.wristMotorKD;

    m_wristMotor.getConfigurator().apply(configWrist);

    m_wristrequest = new VelocityVoltage(0).withSlot(0);
  }

  public void setVelocity(double wvelocity) {
    wristTargetVelocity = wvelocity;
    rollingAvg = 0;

    m_wristMotor.setControl(m_wristrequest.withVelocity(wristTargetVelocity));
  }

  public void setSpeed(double wspeed) {
    wristTargetVelocity = 0;
    rollingAvg = 0;

    m_wristMotor.set(wspeed);
  }

  public void up() {
    this.setSpeed(WristConstants.kTargetWristVelocity);
  }

  public void down() {
    this.setVelocity(WristConstants.wristReverseSpeed);
  }

  public void stop() {
    this.setSpeed(0.0);
  }
}
