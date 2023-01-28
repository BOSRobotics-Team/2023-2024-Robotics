package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default double getDriveScaling() {
    return 1.0;
  }

  public default double getRotateScaling() {
    return 1.0;
  }

  public default Trigger getRobotRelative() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }
}
