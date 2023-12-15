package frc.robot.testsystem;

public abstract interface TestInterface {
    
    public static enum TestStates {

        PASSED, FAILED,

        // - Errors go here -
        NOT_IMPLEMENTED

    };

    public default TestStates Test(String test){
        return TestStates.NOT_IMPLEMENTED;
    }

}
