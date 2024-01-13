package frc.robot.testsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;
import frc.robot.testsystem.TestInterface.TestStates;

public class TestChecklist {

    private ShuffleboardTab m_tab;
    private Map<String, SimpleWidget> m_tests;
    private Map<String, Boolean> m_testStates;

    private RobotContainer m_robot = RobotContainer.GetInstance();
    private Boolean m_enableTestChecklist = false;

    private Boolean m_teleopEnabled = false;
    /** Sets whether Teleop Mode is enabled or disabled. */
    public void SetTeleopEnabled(Boolean _value) { m_teleopEnabled = _value; }
    /** Gets whether Teleop Mode is enabled or disabled. */
    public Boolean GetTeleopEnabled() { return m_teleopEnabled; }

    /** List of subsystems that allow test interfacing. */
    public TestableSubsytem[] subsytems; 

    private List<String> testQueue = new ArrayList<String>();
    /** Gets a list of Test identifiers as Strings. */
    public List<String> GetTestQueue() { return testQueue; }
    /**
     * Adds Test / Tests to the Test Queue
     * 
     * @param Tests A list of Test identifiers to add to the queue
     */
    public void AddTestsToQueue(String... _tests) {
        for (String _test : _tests)
            testQueue.add(_test);
    }
    /**
     * Removes Test / Tests to the Test Queue
     * 
     * @param Tests A list of Test identifiers to remove to the queue
     */
    public void RemoveTestsFromQueue(String... _tests) {
        for (String _test : _tests)
            testQueue.remove(_test);
    }

    /**
     * Sets up a list of subsystems that allow test
     * interfacing and the Test Shuffleboard tab.
     *
     * @param Subsystems List of Testable Subsystems 
    */
    public TestChecklist(TestableSubsytem... _subsytems) { 
        subsytems = _subsytems; 
        // InitializeShuffleBoard();
    }

    /* Test Runner Implementation */
    private TestableSubsytem FindTestSubsytem(String _test) {
        for (TestableSubsytem _subsystem : subsytems) 
            if (_subsystem.GetTests().contains(_test))
                return _subsystem;
        return null;
    }
    private TestStates RunTest(String _test) { 
        m_tests.get(_test).withProperties(Map.of("Color when false", "red"));
        return FindTestSubsytem(_test)
            .Test(_test); 
    }
    private void RunTestQueue() {
        TestStates _state = RunTest(testQueue.get(0));
        if (_state != TestStates.RUNNING) {
            SetShuffleTestCardValue(testQueue.get(0), _state);
            RemoveTestsFromQueue(testQueue.get(0));
        }
    }

    public void initialize() {
        InitializeShuffleBoard();
        m_enableTestChecklist = true;
        m_robot.arm.m_pH.disableCompressor();
    }
    public void periodic() {
        if (m_enableTestChecklist) {

            CheckTestButtons();

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

        m_tab = Shuffleboard.getTab("Checklist");

        for (TestableSubsytem _testableSubsytem : subsytems)
            for (String _test : _testableSubsytem.GetTests()) 
                CreateShuffleTestCard(_test, TestStates.NOT_IMPLEMENTED);
        
    }
    private Boolean HasShuffleTestCard(String _test) {
        return m_tests.containsKey(_test);
    }
    private void CreateShuffleTestCard(String _test, TestStates _state) {

        Integer _column = m_tests.size() != 0 ? 
            (m_tests.size() % Constants.MAX_TEST_COLUMNS) * 2 : 0;
        Integer _row = m_tests.size() != 0 ?
            ((m_tests.size() - (m_tests.size() % Constants.MAX_TEST_COLUMNS)) / Constants.MAX_TEST_COLUMNS) : 0;

        SimpleWidget _newWidget = m_tab.add(_test, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withProperties(Map.of("Color when false", "grey"))
            .withPosition(_column, _row)
            .withSize(2, 1);

        m_testStates.put(_test, false);
        m_tests.put(_test, _newWidget);

    }
    private void SetShuffleTestCardValue(String _key, TestStates _state) {

        if (!HasShuffleTestCard(_key)) {
            CreateShuffleTestCard(_key, _state);
            return;
        }
        
        Boolean _value = false;
        switch (_state) {
            case FAILED:
                _value = false;
                break;
            case PASSED:
                _value = true;
                break;
            case NOT_IMPLEMENTED:
                _value = false;
                break;
        }       
        
        m_tests.get(_value)
            .getEntry()
            .setBoolean(_value);
        
    }
    private void OnTestCardValueChanged(String _test) {
        AddTestsToQueue(_test);
    }
    private void CheckTestButtons() {
        
        for (String _test : m_tests.keySet()) {
            if(m_tests.get(_test).getEntry().getBoolean(false) != m_testStates.get(_test)) {
                m_testStates.put(_test, m_tests.get(_test).getEntry().getBoolean(false));
                OnTestCardValueChanged(_test);
            }
        }

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
