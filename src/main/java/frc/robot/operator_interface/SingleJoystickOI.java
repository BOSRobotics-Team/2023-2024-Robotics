package frc.robot.operator_interface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Class for controlling the robot with a single joystick. */
public class SingleJoystickOI implements OperatorInterface {
  protected final CommandJoystick joystick;
  protected final Trigger[] joystickButtons;

  private double driveScaleFactor = 0.5;
  private boolean updateDriveScale = false;
  private double rotateScaleFactor = 1.0;
  private boolean updateRotateScale = false;
  protected boolean tests[][] = new boolean[2][20];

  public SingleJoystickOI(int port) {
    joystick = new CommandJoystick(port);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.joystickButtons = new Trigger[13];

    for (int i = 1; i < joystickButtons.length; i++) {
      joystickButtons[i] = joystick.button(i);
    }
  }

  public void testController(CommandJoystick contrl, boolean[] test) {
    for (int testNum = 0; testNum < test.length; ++testNum) {
      if (!test[testNum]) {
        switch (testNum) {
          case 0:
            test[testNum] = MathUtil.applyDeadband(contrl.getY(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 1:
            test[testNum] = MathUtil.applyDeadband(contrl.getX(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 2:
            test[testNum] =
                MathUtil.applyDeadband(contrl.getTwist(), Constants.STICK_DEADBAND) > 0.0;
            break;
          case 3:
            test[testNum] = contrl.button(0).getAsBoolean();
            break;
          case 4:
            test[testNum] = contrl.button(1).getAsBoolean();
            break;
          case 5:
            test[testNum] = contrl.button(2).getAsBoolean();
            break;
          case 6:
            test[testNum] = contrl.button(3).getAsBoolean();
            break;
          case 7:
            test[testNum] = contrl.button(4).getAsBoolean();
            break;
          case 8:
            test[testNum] = contrl.button(5).getAsBoolean();
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
      testController(joystick, tests[mode]);

      for (boolean test : tests[mode]) {
        result = result && test;
      }
    }
    return result;
  }

  @Override
  public double getTranslateX() {
    return -joystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -joystick.getX();
  }

  @Override
  public double getRotate() {
    return -joystick.getTwist();
  }

  @Override
  public double getDriveScaling() {
    int povVal = joystick.getHID().getPOV();
    if (updateDriveScale && povVal == -1) {
      updateDriveScale = false;
    }
    if (!updateDriveScale && povVal == 0) {
      driveScaleFactor = MathUtil.clamp(driveScaleFactor + 0.05, 0.1, 1.0);
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateDriveScale = true;
    } else if (!updateDriveScale && povVal == 180) {
      driveScaleFactor = MathUtil.clamp(driveScaleFactor - 0.05, 0.1, 1.0);
      System.out.println("Setting driveScaleFactor to " + driveScaleFactor);
      updateDriveScale = true;
    }
    return driveScaleFactor;
  }

  @Override
  public double getRotateScaling() {
    int povVal = joystick.getHID().getPOV();
    if (updateRotateScale && povVal == -1) {
      updateRotateScale = false;
    }
    if (!updateRotateScale && povVal == 90) {
      rotateScaleFactor = MathUtil.clamp(rotateScaleFactor + 0.05, 0.1, 1.0);
      System.out.println("Setting rotateScaleFactor to " + rotateScaleFactor);
      updateRotateScale = true;
    } else if (!updateRotateScale && povVal == 270) {
      rotateScaleFactor = MathUtil.clamp(rotateScaleFactor - 0.05, 0.1, 1.0);
      System.out.println("Setting rotateScaleFactor to " + rotateScaleFactor);
      updateRotateScale = true;
    }
    return rotateScaleFactor;
  }

  @Override
  public Trigger getRobotRelative() {
    return joystickButtons[4];
  }

  @Override
  public Trigger getResetGyroButton() {
    return joystickButtons[3];
  }

  @Override
  public Trigger getXStanceButton() {
    return joystickButtons[2];
  }
}
