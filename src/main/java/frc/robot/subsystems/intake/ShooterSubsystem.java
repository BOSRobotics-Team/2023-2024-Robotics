package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_topShooterMotor = new TalonFX(ShooterConstants.TOPSHOOTERMOTOR_ID, "Rio");

  private final VelocityVoltage m_topRequest;
  private TalonFXConfiguration configTopShooter = new TalonFXConfiguration();

  private final TalonFX m_bottomShooterMotor =
      new TalonFX(ShooterConstants.BOTTOMSHOOTERMOTOR_ID, "Rio");

  private final VelocityVoltage m_bottomRequest;
  private TalonFXConfiguration configBottomShooter = new TalonFXConfiguration();

  private double topTargetVelocity = 0;
  private double bottomTargetVelocity = 0;
  private double aveTargetVelocity = 0;

  private int rollingAvg = 0;

  // Subsystem Constructor
  public ShooterSubsystem() {

    configTopShooter.Slot0.kS = ShooterConstants.topMotorKS;
    configTopShooter.Slot0.kV = ShooterConstants.topMotorKV;
    configTopShooter.Slot0.kP = ShooterConstants.topMotorKP;
    configTopShooter.Slot0.kI = ShooterConstants.topMotorKI;
    configTopShooter.Slot0.kD = ShooterConstants.topMotorKD;
    configBottomShooter.Slot0.kS = ShooterConstants.bottomMotorKS;
    configBottomShooter.Slot0.kV = ShooterConstants.bottomMotorKV;
    configBottomShooter.Slot0.kP = ShooterConstants.bottomMotorKP;
    configBottomShooter.Slot0.kI = ShooterConstants.bottomMotorKI;
    configBottomShooter.Slot0.kD = ShooterConstants.bottomMotorKD;

    m_bottomShooterMotor.getConfigurator().apply(configBottomShooter);
    m_topShooterMotor.getConfigurator().apply(configTopShooter);

    m_topRequest = new VelocityVoltage(0).withSlot(0);

    m_bottomRequest = new VelocityVoltage(0).withSlot(0);
  }

  public void setVelocity(double tvelocity, double bvelocity) {
    topTargetVelocity = tvelocity;
    bottomTargetVelocity = bvelocity;
    aveTargetVelocity = (topTargetVelocity + bottomTargetVelocity) / 2.0;
    rollingAvg = 0;

    m_topShooterMotor.setControl(m_topRequest.withVelocity(topTargetVelocity));
    m_bottomShooterMotor.setControl(m_bottomRequest.withVelocity(bottomTargetVelocity));
  }

  public void setSpeed(double tspeed, double bspeed) {
    topTargetVelocity = 0;
    bottomTargetVelocity = 0;
    aveTargetVelocity = 0;
    rollingAvg = 0;

    m_topShooterMotor.set(tspeed);
    m_bottomShooterMotor.set(bspeed);
  }

  public void run() {
    this.setVelocity(ShooterConstants.kTargetTopVelocity, ShooterConstants.kTargetBottomVelocity);
  }

  public void run2() {
    this.setVelocity(ShooterConstants.kTargetTopVelocity2, ShooterConstants.kTargetBottomVelocity2);
  }

  public void reverse() {
    this.setSpeed(ShooterConstants.shooterReverseSpeed, ShooterConstants.shooterReverseSpeed);
  }

  public void stop() {
    this.setSpeed(0.0, 0.0);
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
    boolean onTarget = Math.abs(aveTargetVelocity - vel) <= ShooterConstants.velocityTolerance;

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
    SmartDashboard.putNumber("LShooter Target Vel", topTargetVelocity);
    SmartDashboard.putNumber("RShooter Target Vel", bottomTargetVelocity);
  }
}
