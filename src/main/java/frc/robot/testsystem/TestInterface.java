package frc.robot.testsystem;

public abstract interface TestInterface {
    
    /** A bunch of returnable Test-End-States. */
    public static enum TestStates {
        PASSED, FAILED, RUNNING,
        NOT_IMPLEMENTED
    };

    /**
     * Runs every 10ms when this test is run in
     * test mode.
     * 
     * @param test
     * @return The Current State of the Test
     */
    public default TestStates Test(String kTest){
        return TestStates.NOT_IMPLEMENTED;
    }

}
