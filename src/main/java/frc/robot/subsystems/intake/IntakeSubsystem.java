package frc.robot.subsystems.intake;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final SimableCANSparkMax m_intakeMotor =
      new SimableCANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
  private final SparkLimitSwitch m_intakeLimit;
  private final DigitalInput m_intakeSensor = new DigitalInput(IntakeConstants.intakeSensorID);

  private boolean m_useLimit = true;

  // Subsystem Constructor
  public IntakeSubsystem() {

    m_intakeMotor.restoreFactoryDefaults();
    // initialze PID controller and encoder objects
    m_intakeLimit = m_intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_intakeLimit.enableLimitSwitch(false);

    stop();

    m_intakeMotor.burnFlash();
  }

  public void run() {
    m_intakeMotor.set(IntakeConstants.intakeRunSpeed);
  }

  public void reverse() {
    m_intakeMotor.set(IntakeConstants.intakeReverseSpeed);
  }

  public void stop() {
    m_intakeMotor.set(0.0);
  }

  public boolean get_intakeSensor() {
    return m_useLimit ? m_intakeLimit.isPressed() : m_intakeSensor.get();
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake Limit Sensor", m_intakeLimit.isPressed());
    SmartDashboard.putBoolean("Intake Sensor", m_intakeSensor.get());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }
}
