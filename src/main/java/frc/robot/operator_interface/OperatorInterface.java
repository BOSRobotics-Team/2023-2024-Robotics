package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public static int DRIVER = 0;
  public static int OPERATOR = 1;

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default double getRotateY() {
    return 0.0;
  }

  public default boolean isDriveScaling() {
    return false;
  }

  public default Trigger getDriveScaling() {
    return new Trigger(() -> isDriveScaling());
  }

  public default double driveScalingValue() {
    return 1.0;
  }

  public default Trigger getDriveSlowMode() {
    return new Trigger(() -> false);
  }

  public default boolean isRobotRelative() {
    return false;
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

  public default Trigger getRunIntake() {
    return new Trigger(() -> false);
  }

  public default Trigger getShoot() {
    return new Trigger(() -> false);
  }

  public default void testOI(int mode) {}

  public default boolean testResults(int mode) {
    return true;
  }
}
