package frc.robot.testsystem;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TestableSubsytem extends SubsystemBase implements TestInterface {
    
    /** List of tests this subsystem is capable of running. */
    private List<String> tests; 

    /**
     * Creates a Test object that corresponds with a case inside
     * a switch statement in the Override-able "Test" function.
     * 
     * With this approach, you can either create individual tests or 
     * create test groups.
     * 
     * @param name Test identifier
     */
    public void CreateTest(String kName) { tests.add(kName); }

    /** @return A list of tests as Strings. */
    public List<String> GetTests() { return tests; }

}
