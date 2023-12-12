package frc.robot.testsystem;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TestableSubsytem extends SubsystemBase implements TestInterface {
    
    private List<String> tests;
    public void CreateTest(String name) { tests.add(name); }
    public List<String> GetTests() { return tests; }

}
