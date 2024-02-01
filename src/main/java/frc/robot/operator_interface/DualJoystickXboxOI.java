package frc.robot.operator_interface;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualJoystickXboxOI extends DualJoysticksOI {
  private final XboxController operator;

  public DualJoystickXboxOI(int translatePort, int rotatePort, int xboxPort) {
    super(translatePort, rotatePort);
    operator = new XboxController(xboxPort);
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
  public Trigger getRunIntake() {
    return new Trigger(operator::getYButton);
  }

  @Override
  public Trigger getShoot() {
    return new Trigger(operator::getAButton);
  }

  @Override
  public double getLClimber() {
    return -MathUtil.applyDeadband(operator.getLeftY(), STICK_DEADBAND);
  }

  @Override
  public double getRClimber() {
    return -MathUtil.applyDeadband(operator.getRightY(), STICK_DEADBAND);
  }

  @Override
  public boolean testResults(int mode) {
    boolean result = true;
    if (mode == DRIVER) {
      return super.testResults(mode);
    } else if (mode == OPERATOR) {
      testController(operator, super.tests[2]);

      for (boolean test : tests[2]) {
        result = result && test;
      }
    }
    return result;
  }
}
