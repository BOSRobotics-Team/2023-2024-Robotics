package frc.robot.operator_interface;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;
  private double driveScaleFactor = 0.5;
  private boolean updateDriveScale = false;
  protected boolean tests[][] = new boolean[2][20];

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  public void testController(XboxController contrl, boolean[] test) {
    for (int testNum = 0; testNum < test.length; ++testNum) {
      if (!test[testNum]) {
        switch (testNum) {
          case 0:
            test[testNum] = MathUtil.applyDeadband(contrl.getLeftY(), STICK_DEADBAND) > 0.0;
            break;
          case 1:
            test[testNum] = MathUtil.applyDeadband(contrl.getLeftX(), STICK_DEADBAND) > 0.0;
            break;
          case 2:
            test[testNum] = MathUtil.applyDeadband(contrl.getRightX(), STICK_DEADBAND) > 0.0;
            break;
          case 3:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getLeftTriggerAxis(), STICK_DEADBAND) > 0.0;
            break;
          case 4:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getRightTriggerAxis(), STICK_DEADBAND) > 0.0;
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
    return -MathUtil.applyDeadband(controller.getLeftY(), STICK_DEADBAND);
  }

  @Override
  public double getTranslateY() {
    return -MathUtil.applyDeadband(controller.getLeftX(), STICK_DEADBAND);
  }

  @Override
  public double getRotate() {
    return -MathUtil.applyDeadband(controller.getRightX(), STICK_DEADBAND);
  }

  @Override
  public double getRotateY() {
    return -MathUtil.applyDeadband(controller.getRightY(), STICK_DEADBAND);
  }

  @Override
  public boolean isDriveScaling() {
    int povVal = controller.getPOV();

    if (updateDriveScale) {
      updateDriveScale = (povVal != -1);
    } else if (povVal == 0) {
      driveScaleFactor = MathUtil.clamp(driveScaleFactor + 0.05, 0.1, 1.0);
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateDriveScale = true;
    } else if (povVal == 180) {
      driveScaleFactor = MathUtil.clamp(driveScaleFactor - 0.05, 0.1, 1.0);
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateDriveScale = true;
    }
    return updateDriveScale;
  }

  @Override
  public Trigger getDriveScaling() {
    return new Trigger(() -> isDriveScaling());
  }

  @Override
  public double driveScalingValue() {
    return driveScaleFactor;
  }

  @Override
  public boolean isRobotRelative() {
    return controller.getLeftBumper();
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
  public Trigger getRunIntake() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getShoot() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public double getLClimber() {
    return MathUtil.applyDeadband(controller.getLeftTriggerAxis(), STICK_DEADBAND);
  }

  @Override
  public double getRClimber() {
    return MathUtil.applyDeadband(controller.getRightTriggerAxis(), STICK_DEADBAND);
  }

}
