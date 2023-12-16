package frc.robot.testsystem;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;
import frc.robot.testsystem.TestInterface.TestStates;

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
    private TestStates RunTest(String kTest) { 
        return FindTestSubsytem(kTest)
            .Test(kTest); 
    }

    /**
     * This runs every 10ms and runs the test that is at the
     * top of the queue.
     * 
     * You can find the identifier of the current test being run by 
     * using "GetTestQueue().get(0);".
     */
    public void RunTestQueue() {
        if (RunTest(testQueue.get(0)) != TestStates.RUNNING)
            RemoveTestsFromQueue(testQueue.get(0));
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
