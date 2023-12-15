package frc.robot.testsystem;

public abstract interface TestInterface {
    
    /** A bunch of returnable Test-End-States. */
    public static enum TestEndStates {
        PASSED, FAILED,
        NOT_IMPLEMENTED
    };

    /**
     * Runs every 10ms when this test is run in
     * test mode.
     * 
     * @param test
     * @return
     */
    public default TestEndStates Test(String test){
        return TestEndStates.NOT_IMPLEMENTED;
    }

}
