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

  public default double getArmLift() {
    return 0.0;
  }

  public default double getArmExtend() {
    return 0.0;
  }

  public default Trigger getArmCalibrate() {
    return new Trigger(() -> false);
  }

  public default Trigger getGripToggle() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmPosition0() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmPosition1() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmPosition2() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmPosition3() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmTargetToggle() {
    return new Trigger(() -> false);
  }

  public default void testOI(int mode) {}

  public default boolean testResults(int mode) {
    return true;
  }
}
