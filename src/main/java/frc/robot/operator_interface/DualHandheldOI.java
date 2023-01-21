package frc.robot.operator_interface;

// import edu.wpi.first.wpilibj.XboxController;

/** Class for controlling the robot with a dual Xbox controllers. */
public class DualHandheldOI extends SingleHandheldOI {
  // private final XboxController operator;

  public DualHandheldOI(int port0, int port1) {
    super(port0);
    // operator = new XboxController(port1);
  }
}
