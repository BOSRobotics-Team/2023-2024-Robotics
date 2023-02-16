package frc.robot.operator_interface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;
  private double driveScaleFactor = 0.5;
  private double rotateScaleFactor = 1.0;
  private boolean updateScale = false;
  protected boolean tests[][] = new boolean[2][17];

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  public void testController(XboxController contrl, boolean[] test) {
    for (int testNum = 0; testNum < test.length; ++testNum) {
      if (!test[testNum]) {
        switch (testNum) {
          case 0:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getLeftY(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 1:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getLeftX(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 2:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getRightX(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 3:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getLeftTriggerAxis(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 4:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getRightTriggerAxis(), Constants.STICK_DEADBAND)
                    > 0.0;
            break;
          case 5:
            test[testNum] = contrl.getPOV() == 0;
            break;
          case 6:
            test[testNum] = contrl.getPOV() == 90;
            break;
          case 7:
            test[testNum] = contrl.getPOV() == 180;
            break;
          case 8:
            test[testNum] = contrl.getPOV() == 270;
            break;
          case 9:
            test[testNum] = contrl.getLeftBumper();
            break;
          case 10:
            test[testNum] = contrl.getRightBumper();
            break;
          case 11:
            test[testNum] = contrl.getAButton();
            break;
          case 12:
            test[testNum] = contrl.getBButton();
            break;
          case 13:
            test[testNum] = contrl.getXButton();
            break;
          case 14:
            test[testNum] = contrl.getYButton();
            break;
          case 15:
            test[testNum] = contrl.getStartButton();
            break;
          case 16:
            test[testNum] = contrl.getBackButton();
            break;
          default:
            test[testNum] = true;
            break;
        }
      }
    }
  }

  @Override
  public void testOI(int mode) {
    for (int testNum = 0; testNum < tests[mode].length; ++testNum) {
      tests[mode][testNum] = false;
    }
  }

  @Override
  public boolean testResults(int mode) {
    boolean result = true;
    if (mode == DRIVER) {
      testController(controller, tests[mode]);

      for (boolean test : tests[mode]) {
        result = result && test;
      }
    }
    return result;
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    return -controller.getRightX();
  }

  @Override
  public double getDriveScaling() {
    int povVal = controller.getPOV();

    if (updateScale && povVal == -1) {
      updateScale = false;
    }
    if (!updateScale && povVal == 0) {
      driveScaleFactor += 0.05;
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateScale = true;
    } else if (!updateScale && povVal == 180) {
      driveScaleFactor -= 0.05;
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateScale = true;
    }
    driveScaleFactor = MathUtil.clamp(driveScaleFactor, 0.1, 1.0);
    return driveScaleFactor;
  }

  @Override
  public double getRotateScaling() {
    int povVal = controller.getPOV();

    if (updateScale && povVal == -1) {
      updateScale = false;
    }
    if (!updateScale && povVal == 90) {
      rotateScaleFactor += 0.05;
      System.out.println("Setting rotateScaleFactor to " + rotateScaleFactor);
      updateScale = true;
    } else if (!updateScale && povVal == 270) {
      rotateScaleFactor -= 0.05;
      System.out.println("Setting rotateScaleFactor to " + rotateScaleFactor);
      updateScale = true;
    }
    rotateScaleFactor = MathUtil.clamp(rotateScaleFactor, 0.1, 1.0);
    return rotateScaleFactor;
  }

  @Override
  public Trigger getRobotRelative() {
    return new Trigger(controller::getLeftBumper);
  }

  @Override
  public Trigger getResetGyroButton() {
    return new Trigger(controller::getStartButton);
  }

  @Override
  public Trigger getXStanceButton() {
    return new Trigger(controller::getRightBumper);
  }

  @Override
  public Trigger getGripToggle() {
    return new Trigger(() -> controller.getLeftTriggerAxis() != 0.0);
  }

  @Override
  public Trigger getArmPosition0() {
    return new Trigger(controller::getAButton);
  }

  public Trigger getArmPosition1() {
    return new Trigger(controller::getXButton);
  }

  public Trigger getArmPosition2() {
    return new Trigger(controller::getYButton);
  }

  public Trigger getArmPosition3() {
    return new Trigger(controller::getBButton);
  }
}
