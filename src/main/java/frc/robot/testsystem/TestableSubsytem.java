package frc.robot.testsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public abstract class TestableSubsytem extends SubsystemBase implements TestInterface {

  /** List of tests this subsystem is capable of running. */
  private List<String> tests = new ArrayList<String>();

  /**
   * Creates a Test object that corresponds with a case inside a switch statement in the
   * Override-able "Test" function.
   *
   * <p>With this approach, you can either create individual tests or create test groups.
   *
   * @param name Test identifier
   */
  public void CreateTest(String _name) {
    try {
      tests.add(_name);
    } catch (NullPointerException e) {
      System.out.println("| Tests was not initialized >> " + e.getMessage());
    }
  }

  /**
   * @return A list of tests as Strings.
   */
  public List<String> GetTests() {
    return tests;
  }
}
