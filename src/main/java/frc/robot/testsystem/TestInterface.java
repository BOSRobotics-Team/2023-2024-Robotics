package frc.robot.testsystem;

public abstract interface TestInterface {

  /** A bunch of returnable Test-End-States. */
  public static enum TestStates {
    PASSED,
    FAILED,
    RUNNING,
    NOT_IMPLEMENTED
  };

  /**
   * Runs every 10ms when this test is run in test mode.
   *
   * @param test The Test you want to run
   * @return The Current State of the Test
   */
  public default TestStates Test(String _test) {
    return TestStates.NOT_IMPLEMENTED;
  }
}
