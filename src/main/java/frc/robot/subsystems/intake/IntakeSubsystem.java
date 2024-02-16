package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_intakeMotor =
      new SimableCANSparkMax(IntakeConstants.INTAKEMOTOR_ID, MotorType.kBrushless);
  private final SparkLimitSwitch m_intakeLimit;

  public boolean m_interuptIntake = false;

  // Subsystem Constructor
  public IntakeSubsystem() {

    m_intakeMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_intakeLimit = m_intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_intakeLimit.enableLimitSwitch(false);
    m_intakeMotor.setInverted(true);

    stop();

    m_intakeMotor.burnFlash();
  }

  public void run() {
    m_intakeMotor.set(IntakeConstants.intakeRunSpeed);
  }

  public void reverse() {
    m_interuptIntake = true;
    m_intakeMotor.set(IntakeConstants.intakeReverseSpeed);
  }

  public void stop() {
    m_interuptIntake = false;
    m_intakeMotor.set(0.0);
  }

  public boolean get_intakeSensor() {
    return m_intakeLimit.isPressed();
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake Limit Sensor", m_intakeLimit.isPressed());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }
}
