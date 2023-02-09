package frc.robot.operator_interface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;
  private double driveScaleFactor = 0.5;
  private double rotateScaleFactor = 1.0;
  private boolean updateScale = false;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
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
    return new Trigger(controller::getYButton);
  }

  @Override
  public double getArmLift() {
    return controller.getLeftTriggerAxis();
  }

  @Override
  public double getArmExtend() {
    return controller.getRightTriggerAxis();
  }
  @Override
  public Trigger getGripToggle() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getArmCalibrate() {
    return new Trigger(controller::getBackButton);
  }

}
