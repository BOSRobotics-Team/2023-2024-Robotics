package frc.robot.testsystem;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;
import frc.robot.testsystem.TestInterface.TestStates;

public class TestChecklist {
    
    private ShuffleboardTab m_tab = Shuffleboard.getTab("Checklist");
    private Map<String, SimpleWidget> m_tests;
    
    private final Integer MAX_COLUMNS = 10;
    private Integer m_currentColumn = 0;

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
        m_tests.get(kTest).withProperties(Map.of("Color when false", "red"));
        return FindTestSubsytem(kTest)
            .Test(kTest); 
    }

    private void RunTestQueue() {
        TestStates kState = RunTest(testQueue.get(0));
        if (kState != TestStates.RUNNING) {
            SetShuffleTestCardValue(testQueue.get(0), kState);
            RemoveTestsFromQueue(testQueue.get(0));
        }
    }

    public void initialize() {
        m_enableTestChecklist = true;
        m_robot.arm.m_pH.disableCompressor();
    }
    public void periodic() {
        if (m_enableTestChecklist) {
            if (GetTestQueue().size() > 0)
                RunTestQueue();
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
        for (TestableSubsytem kTestableSubsytem : subsytems)
            for (String kTest : kTestableSubsytem.GetTests())
                CreateShuffleTestCard(kTest, TestStates.NOT_IMPLEMENTED);
    }
    private Boolean HasShuffleTestCard(String kTest) {
        return m_tests.containsKey(kTest);
    }
    private void CreateShuffleTestCard(String kTest, TestStates kState) {

        Integer kColumn = m_tests.size() != 0 ? (m_tests.size() % MAX_COLUMNS) + 1 : 0;
        Integer kRow = m_tests.size() != 0 ? ((m_tests.size() - (m_tests.size() % MAX_COLUMNS)) / MAX_COLUMNS) : 0;

        SimpleWidget kNewWidget = m_tab.add(kTest, false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when false", "grey"))
            .withPosition(kColumn, kRow)
            .withSize(2, 1);

        m_tests.put(kTest, kNewWidget);

    }
    private void SetShuffleTestCardValue(String kKey, TestStates kState) {

        if (!HasShuffleTestCard(kKey)) {
            CreateShuffleTestCard(kKey, kState);
            return;
        }
        
        Boolean kValue = false;
        switch (kState) {
            case FAILED:
                kValue = false;
                break;
            case PASSED:
                kValue = true;
                break;
            case NOT_IMPLEMENTED:
                kValue = false;
                break;
        }       
        
        m_tests.get(kValue)
            .getEntry()
            .setBoolean(kValue);
        
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
