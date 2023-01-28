package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for selecting the appropriate OI implementations based on the connected joysticks.
 */
public class OISelector {
  private static String[] lastJoystickNames = new String[] {null, null, null, null, null, null};
  private static final String noOperatorInterfaceWarning = "No operator controller(s) connected.";
  private static final String dualJoystickOperatorInterfaces = "Dual Joysticks connected.";
  private static final String singleJoystickOperatorInterfaces = "Single Joystick connected.";
  private static final String dualXBoxOperatorInterfaces = "Dual XBox operator controllers connected.";
  private static final String singleXBoxOperatorInterfaces = "XBox operator controller connected.";

  private OISelector() {}

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;

    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port);
      if (!name.equals(lastJoystickNames[port])) {
        lastJoystickNames[port] = name;
        joysticksChanged = true;
      }
    }
    return joysticksChanged;
  }

  /**
   * Instantiates and returns an appropriate OperatorInterface object based on the connected
   * joysticks.
   */
  public static OperatorInterface findOperatorInterface() {
    Integer joy0 = null;
    Integer joy1 = null;
    Integer xbox0 = null;
    Integer xbox1 = null;

    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.getJoystickName(port).toLowerCase().contains("xbox")) {
        if (xbox0 == null) {
          xbox0 = port;
        } else if (xbox1 == null) {
          xbox1 = port;
        }
      } else if (!DriverStation.getJoystickName(port).equals("")) {
        if (joy0 == null) {
          joy0 = port;
        } else if (joy1 == null) {
          joy1 = port;
        }
      }
    }

    if (joy0 != null && joy1 != null) {
      System.out.println(dualJoystickOperatorInterfaces);
      return new DualJoysticksOI(joy0, joy1);
    } else if (xbox0 != null && xbox1 != null) {
      System.out.println(dualXBoxOperatorInterfaces);
      return new DualHandheldOI(xbox0, xbox1);
    } else if (joy0 != null) {
      System.out.println(singleJoystickOperatorInterfaces);
      return new SingleHandheldOI(joy0) {};
    } else if (xbox0 != null) {
      System.out.println(singleXBoxOperatorInterfaces);
      return new SingleHandheldOI(xbox0) {};
    } else {
      DriverStation.reportWarning(noOperatorInterfaceWarning, false);
      return new OperatorInterface() {};
    }
  }    
}
