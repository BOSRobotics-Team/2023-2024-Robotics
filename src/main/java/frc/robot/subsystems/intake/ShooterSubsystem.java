package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final VictorSPX m_aimMotor = new VictorSPX(ShooterConstants.AIMMOTOR_ID);

  private final SimableCANSparkMax m_leftShooterMotor =
      new SimableCANSparkMax(ShooterConstants.LEFTSHOOTERMOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_leftShooterEncoder;
  private final SparkPIDController m_leftShooterController;

  private final SimableCANSparkMax m_rightShooterMotor =
      new SimableCANSparkMax(ShooterConstants.RIGHTSHOOTERMOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_rightShooterEncoder;
  private final SparkPIDController m_rightShooterController;

  private double leftTargetVelocity = 0;
  private double rightTargetVelocity = 0;
  private double aveTargetVelocity = 0;

  private int rollingAvg = 0;

  // Subsystem Constructor
  public ShooterSubsystem() {

    m_leftShooterMotor.setInverted(true);
    m_rightShooterMotor.setInverted(false);

    m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
    m_rightShooterEncoder = m_rightShooterMotor.getEncoder();

    m_leftShooterController = m_leftShooterMotor.getPIDController();
    m_rightShooterController = m_rightShooterMotor.getPIDController();

    m_leftShooterController.setP(ShooterConstants.proportialPIDConstant);
    m_leftShooterController.setI(ShooterConstants.integralPIDConstant);
    m_leftShooterController.setD(ShooterConstants.derivativePIDConstant);
    m_leftShooterController.setIZone(ShooterConstants.integralPIDConstant);
    m_leftShooterController.setFF(ShooterConstants.leftFeedForwardPIDConstant);
    m_leftShooterController.setOutputRange(
        ShooterConstants.minPIDOutput, ShooterConstants.maxPIDOutput);

    m_rightShooterController.setP(ShooterConstants.proportialPIDConstant);
    m_rightShooterController.setI(ShooterConstants.integralPIDConstant);
    m_rightShooterController.setD(ShooterConstants.derivativePIDConstant);
    m_rightShooterController.setIZone(ShooterConstants.integralPIDConstant);
    m_rightShooterController.setFF(ShooterConstants.rightFeedForwardPIDConstant);
    m_rightShooterController.setOutputRange(
        ShooterConstants.minPIDOutput, ShooterConstants.maxPIDOutput);
    stop();

    m_leftShooterMotor.burnFlash();
    m_rightShooterMotor.burnFlash();
  }

  public void runAimMotor(double percent) {
    m_aimMotor.set(
        VictorSPXControlMode.PercentOutput, -percent); // negative because motor is upside down
  }

  public void stopAimMotor() {
    m_aimMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void setVelocity(double lvelocity, double rvelocity) {
    leftTargetVelocity = lvelocity;
    rightTargetVelocity = rvelocity;
    aveTargetVelocity = (leftTargetVelocity + rightTargetVelocity) / 2.0;
    rollingAvg = 0;

    m_leftShooterController.setReference(leftTargetVelocity, ControlType.kVelocity);
    m_rightShooterController.setReference(rightTargetVelocity, ControlType.kVelocity);
  }

  public void setSpeed(double lspeed, double rspeed) {
    leftTargetVelocity = 0;
    rightTargetVelocity = 0;
    aveTargetVelocity = 0;
    rollingAvg = 0;

    m_leftShooterMotor.set(lspeed);
    m_rightShooterMotor.set(rspeed);
  }

  public void run() {
    this.setVelocity(ShooterConstants.kTargetLeftVelocity, ShooterConstants.kTargetRightVelocity);
  }

  public void run2() {
    this.setVelocity(ShooterConstants.kTargetLeftVelocity2, ShooterConstants.kTargetRightVelocity2);
  }

  public void reverse() {
    this.setSpeed(ShooterConstants.shooterReverseSpeed, ShooterConstants.shooterReverseSpeed);
  }

  public void stop() {
    this.setSpeed(0.0, 0.0);
  }

  // Finds the average velocity of the two motors
  public double getVelocity() {
    double sum = m_leftShooterEncoder.getVelocity() + m_rightShooterEncoder.getVelocity();
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

    SmartDashboard.putNumber("LShooter Vel", m_leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("RShooter Vel", m_rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Ave Shooter Vel", getVelocity());
    // SmartDashboard.putBoolean("Launcher On Target", isOnTarget());
    SmartDashboard.putBoolean("Avg Shooter OnTarget", isOnTargetAverage(7));
    SmartDashboard.putNumber("LShooter Target Vel", leftTargetVelocity);
    SmartDashboard.putNumber("RShooter Target Vel", rightTargetVelocity);
  }
}
