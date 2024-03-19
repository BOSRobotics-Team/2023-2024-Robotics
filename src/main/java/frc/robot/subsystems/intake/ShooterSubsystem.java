package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_topShooterMotor =
      new TalonFX(ShooterConstants.TOPSHOOTERMOTOR_ID, "CANivore");
  private final TalonFX m_bottomShooterMotor =
      new TalonFX(ShooterConstants.BOTTOMSHOOTERMOTOR_ID, "CANivore");

  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private final VelocityVoltage m_voltageVelocity =
      new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  /* Start at velocity 0, no feed forward, use slot 1 */
  // private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0,
  // 1, false, false, false);
  /* Keep a neutral out so we can disable the motor */
  // private final NeutralOut m_brake = new NeutralOut();

  private double targetVelocity = 0;
  private int rollingAvg = 0;

  // Subsystem Constructor
  public ShooterSubsystem() {

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.Slot0.kS = ShooterConstants.KSConstant;
    configs.Slot0.kP = ShooterConstants.proportialPIDConstant;
    configs.Slot0.kI = ShooterConstants.integralPIDConstant;
    configs.Slot0.kD = ShooterConstants.derivativePIDConstant;
    configs.Slot0.kV = ShooterConstants.feedForwardPIDConstant;
    configs.Voltage.PeakForwardVoltage = ShooterConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ShooterConstants.peakReverseVoltage;

    configs.Slot1.kP = ShooterConstants.proportialTorquePIDConstant;
    configs.Slot1.kI = ShooterConstants.integralTorquePIDConstant;
    configs.Slot1.kD = ShooterConstants.derivativeTorquePIDConstant;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = ShooterConstants.peakReverseTorqueCurrent;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = m_topShooterMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    status = m_bottomShooterMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply bottom configs, error code: " + status.toString());
    }
  }

  public void setVelocity(double velocity) {
    targetVelocity = velocity;
    rollingAvg = 0;

    m_topShooterMotor.setControl(m_voltageVelocity.withVelocity(targetVelocity));
    m_bottomShooterMotor.setControl(m_voltageVelocity.withVelocity(targetVelocity));
    // m_topShooterMotor.setControl(m_torqueVelocity.withVelocity(targetVelocity).withFeedForward(1.0));
    // m_bottomShooterMotor.setControl(m_torqueVelocity.withVelocity(targetVelocity).withFeedForward(1.0));
  }

  public void setSpeed(double speed) {
    targetVelocity = 0;
    rollingAvg = 0;

    m_topShooterMotor.set(speed);
    m_bottomShooterMotor.set(speed);
  }

  public void intake() {
    this.setSpeed(ShooterConstants.shooterReverseSpeed);
  }

  public void reverseintake() {
    this.setSpeed(-ShooterConstants.shooterReverseSpeed);
  }

  public void shoot() {
    // this.setSpeed(1.0);
    this.setVelocity(ShooterConstants.kTargetVelocity);
  }

  public void shoot2() {
    this.setVelocity(ShooterConstants.kTargetVelocity2);
  }

  public void stop() {
    this.setSpeed(0.0);
  }

  // Finds the average velocity of the two motors
  public double getVelocity() {
    double sum =
        m_topShooterMotor.getVelocity().getValue() + m_bottomShooterMotor.getVelocity().getValue();
    double average = sum / 2;

    return average;
  }

  // For the target velocity
  public boolean isOnTarget() {
    double vel = this.getVelocity();
    boolean onTarget = Math.abs(targetVelocity - vel) <= ShooterConstants.velocityTolerance;

    return onTarget;
  }

  public boolean isOnTargetAverage(int percent) {
    return (rollingAvg >= MathUtil.clamp(percent, 0, 10));
  }

  @Override
  public void periodic() {
    if (isOnTarget()) {
      if (rollingAvg < 10) {
        rollingAvg++;
      }
    } else if (rollingAvg > 0) {
      rollingAvg--;
    }
    updateSmartDashboard();
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {

    SmartDashboard.putNumber("LShooter Vel", m_topShooterMotor.getVelocity().getValue());
    SmartDashboard.putNumber("RShooter Vel", m_bottomShooterMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Ave Shooter Vel", getVelocity());
    // SmartDashboard.putBoolean("Launcher On Target", isOnTarget());
    SmartDashboard.putBoolean("Avg Shooter OnTarget", isOnTargetAverage(7));
    SmartDashboard.putNumber("Shooter Target Vel", targetVelocity);
  }
}
