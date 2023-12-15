package frc.robot.testsystem;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;
import frc.robot.testsystem.TestInterface.TestEndStates;

public class TestChecklist {
    
    private RobotContainer m_robot = RobotContainer.GetInstance();
    private Boolean m_enableTestChecklist = false;

    private Boolean m_teleopEnabled = false;
    /** Sets whether Teleop Mode is enabled or disabled. */
    public void SetTeleopEnabled(Boolean kValue) { m_teleopEnabled = kValue; }
    /** Gets whether Teleop Mode is enabled or disabled. */
    public Boolean GetTeleopEnabled() { return m_teleopEnabled; }

    /** List of subsystems that allow test interfacing. */
    public TestableSubsytem[] subsytems; 

    private List<String> testQueue;
    /** Gets a list of Test identifiers as Strings. */
    public List<String> GetTestQueue() { return testQueue; }
    /**
     * Adds Test / Tests to the Test Queue
     * 
     * @param kTests A list of Test identifiers to add to the queue
     */
    public void AddTestsToQueue(String... kTests) {
        for (String kTest : kTests)
            testQueue.add(kTest);
    }
    /**
     * Removes Test / Tests to the Test Queue
     * 
     * @param kTests A list of Test identifiers to remove to the queue
     */
    public void RemoveTestsFromQueue(String... kTests) {
        for (String kTest : kTests)
            testQueue.remove(kTest);
    }

    /**
     * Sets up a list of subsystems that allow test
     * interfacing and the Test Shuffleboard tab.
     *
     * @param subsystems List of Testable Subsystems
     * @return
    */
    public TestChecklist(TestableSubsytem... kSubsytems) { 
        subsytems = kSubsytems; 
        InitializeShuffleBoard();
    }

    /* Test Runner Implementation */
    private TestableSubsytem FindTestSubsytem(String kTest) {
        for (TestableSubsytem kSubsystem : subsytems) 
            if (kSubsystem.GetTests().contains(kTest))
                return kSubsystem;
        return null;
    }
    private TestEndStates RunTest(String kTest) { 
        return FindTestSubsytem(kTest)
            .Test(kTest); 
    }
    private void RunTestQueue(String... kTests) {
        for (String kTest : kTests) {
            TestEndStates state = RunTest(kTest);
            if (state != TestEndStates.PASSED && state != TestEndStates.FAILED) {
                System.out.println("Error in Test : " + kTest + " | " + state.toString());
                // Do stuff here | Returned an Error
            } else if (state == TestEndStates.PASSED) {
                // Do stuff here | Passed
            } else {
                // Do Stuff here | Failed
            }
            
        }
    }

    public void initialize() {
        m_enableTestChecklist = true;
        m_robot.arm.m_pH.disableCompressor();
    }
    public void periodic() {
        if (m_enableTestChecklist) { 
            if (GetTeleopEnabled())
                RunTeleop();
        }
    }
    public void exit() {
        m_enableTestChecklist = false;
        m_robot.arm.m_pH.enableCompressorDigital();
    }

    /* Custom Teleop Mode Implementation */ 
    private void InitializeShuffleBoard(){
        // Do Shuffle board stuff here
    }
    public void RunTeleop() {

        double liftVal = MathUtil.applyDeadband(m_robot.oi.getArmLift(), Constants.STICK_DEADBAND);
        double extendVal = MathUtil.applyDeadband(m_robot.oi.getArmExtend(), Constants.STICK_DEADBAND);
        m_robot.arm.teleop(liftVal, extendVal);
    
        double maxSpeed = DriveTrainConstants.maxSpeed * m_robot.oi.getDriveScaling();
        double maxRotate = DriveTrainConstants.maxAngularVelocity * m_robot.oi.getRotateScaling();
        double translationVal = MathUtil.applyDeadband(m_robot.oi.getTranslateY(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(m_robot.oi.getTranslateY(), Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(m_robot.oi.getRotate(), Constants.STICK_DEADBAND);
    
        m_robot.driveTrain.drive(
            Math.copySign(translationVal * translationVal, translationVal) * maxSpeed,
            Math.copySign(strafeVal * strafeVal, strafeVal) * maxSpeed,
            Math.copySign(rotationVal * rotationVal, rotationVal) * maxRotate
        );

    }
    
}
