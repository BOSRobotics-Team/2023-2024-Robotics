package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single joystick. */
public class SingleJoystickOI implements OperatorInterface {
  private final CommandJoystick joystick;
  private final Trigger[] joystickButtons;

  public SingleJoystickOI(int port) {
    joystick = new CommandJoystick(port);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.joystickButtons = new Trigger[13];

    for (int i = 1; i < joystickButtons.length; i++) {
      joystickButtons[i] = joystick.button(i);
    }
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
    return joystick.getThrottle();
  }

  @Override
  public Trigger getRobotRelative() {
    return joystickButtons[3];
  }

  @Override
  public Trigger getResetGyroButton() {
    return joystickButtons[2];
  }

  @Override
  public Trigger getXStanceButton() {
    return joystickButtons[1];
  }
}
