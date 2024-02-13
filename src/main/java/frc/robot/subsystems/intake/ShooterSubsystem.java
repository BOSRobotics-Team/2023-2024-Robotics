package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_leftShooterMotor =
      new SimableCANSparkMax(IntakeConstants.LEFTSHOOTER_ID, MotorType.kBrushless);
  private final RelativeEncoder m_leftShooterEncoder;
  private final SparkPIDController m_leftShooterController;

  private final SimableCANSparkMax m_rightShooterMotor =
      new SimableCANSparkMax(IntakeConstants.RIGHTSHOOTER_ID, MotorType.kBrushless);
  private final RelativeEncoder m_rightShooterEncoder;
  private final SparkPIDController m_rightShooterController;

  private double leftTargetVelocity = 0;
  private double rightTargetVelocity = 0;

  private int rollingAvg = 0;

  // Subsystem Constructor
  public ShooterSubsystem() {

    m_rightShooterMotor.setInverted(true);

    m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
    m_rightShooterEncoder = m_rightShooterMotor.getEncoder();

    m_leftShooterController = m_leftShooterMotor.getPIDController();
    m_rightShooterController = m_rightShooterMotor.getPIDController();

    m_leftShooterController.setP(IntakeConstants.proportialPIDConstant);
    m_leftShooterController.setI(IntakeConstants.integralPIDConstant);
    m_leftShooterController.setD(IntakeConstants.derivativePIDConstant);
    m_leftShooterController.setIZone(IntakeConstants.integralPIDConstant);
    m_leftShooterController.setFF(IntakeConstants.leftFeedForwardPIDConstant);
    m_leftShooterController.setOutputRange(
        IntakeConstants.minPIDOutput, IntakeConstants.maxPIDOutput);

    m_rightShooterController.setP(IntakeConstants.proportialPIDConstant);
    m_rightShooterController.setI(IntakeConstants.integralPIDConstant);
    m_rightShooterController.setD(IntakeConstants.derivativePIDConstant);
    m_rightShooterController.setIZone(IntakeConstants.integralPIDConstant);
    m_rightShooterController.setFF(IntakeConstants.rightFeedForwardPIDConstant);
    m_rightShooterController.setOutputRange(
        IntakeConstants.minPIDOutput, IntakeConstants.maxPIDOutput);
    stop();

    m_leftShooterMotor.burnFlash();
    m_rightShooterMotor.burnFlash();
  }

  public void setVelocity(double lvelocity, double rvelocity) {
    leftTargetVelocity = lvelocity;
    rightTargetVelocity = rvelocity;

    m_leftShooterController.setReference(leftTargetVelocity, ControlType.kVelocity);
    m_rightShooterController.setReference(rightTargetVelocity, ControlType.kVelocity);
  }

  public void setSpeed(double lspeed, double rspeed) {
    m_leftShooterMotor.set(lspeed);
    m_rightShooterMotor.set(rspeed);
  }

  public void reverse() {
    m_leftShooterMotor.set(IntakeConstants.intakeReverseSpeed);
    m_rightShooterMotor.set(IntakeConstants.intakeReverseSpeed);
  }

  public void run() {
    setVelocity(IntakeConstants.kTargetLeftVelocity, IntakeConstants.kTargetRightVelocity);
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
    boolean leftOnTarget =
        Math.abs(leftTargetVelocity - m_leftShooterEncoder.getVelocity())
            <= IntakeConstants.velocityPIDTolerance;
    boolean rightOnTarget =
        Math.abs(rightTargetVelocity - m_rightShooterEncoder.getVelocity())
            <= IntakeConstants.velocityPIDTolerance;
    return (rightOnTarget && leftOnTarget);
  }

  public boolean isOnTargetAverage(int percent) {
    return (rollingAvg >= MathUtil.clamp(percent, 0, 10));
  }

  public static double distanceToVelocity(double distance) {
    // TODO tune distance convertion
    return 0.0;
  }

  @Override
  public void periodic() {
    if (isOnTarget()) {
      if (rollingAvg < 10) {
        rollingAvg++;
      }
    } else if (rollingAvg > 0) {
      if (rollingAvg > 0) {
        rollingAvg--;
      }
    }
    updateSmartDashboard();
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {

    SmartDashboard.putNumber("Left Velocity", m_leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Average Velocity", getVelocity());
    SmartDashboard.putBoolean("Launcher On Target", isOnTarget());
    SmartDashboard.putBoolean("Avg Launcher On Target", isOnTargetAverage(7));
    SmartDashboard.putNumber("Left Target Velocity", leftTargetVelocity);
    SmartDashboard.putNumber("Right Target Velocity", rightTargetVelocity);
  }
}
