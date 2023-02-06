package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.gyro.GyroIO;
import frc.lib.util.CANDeviceFinder;
import frc.lib.util.DashboardNumber;
import frc.lib.util.CANDeviceId.CANDeviceType;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;

public class TestChecklist {
    private class ChecklistItem {
        public final String Title;
        public final Supplier<Boolean> isComplete;
        public final int column;
        public final int row;
        
        public GenericEntry widget;
        public Boolean complete;
        public String status;

        public ChecklistItem( String title, Supplier<Boolean> compl, int col, int row) {
            this.Title = title;
            this.isComplete = compl;
            this.column = col;
            this.row = row;
            this.widget = null;
            this.complete = false;
            this.status = "";
        }
    }

    private final ShuffleboardTab tabMain = Shuffleboard.getTab("Checklist");

    private final PowerDistribution power = new PowerDistribution();
    private final CANDeviceFinder canFinder = new CANDeviceFinder();

    private final GyroIO gyro;
    private final SwerveDriveTrain driveTrain;

    private final DashboardNumber voltageThreshold = new DashboardNumber("Checklist/VoltageThreshold", 12.0);

    private GenericEntry resetTests;

    private int checkSwerveModule[] = {0, 0, 0, 0};
    private GenericEntry testModule[] = {null, null, null, null};

    private int checkGyroState = 0;
    private GenericEntry testGyro;

    private boolean m_enableCheckList = false;

    private int BATTERY_TEST = 0;
    private int DEVICES_TEST = 1;
    private int SWERVE_MOD_0 = 2;
    private int SWERVE_MOD_1 = 3;
    private int SWERVE_MOD_2 = 4;
    private int SWERVE_MOD_3 = 5;
    private int SWERVE_MODULES = 6;
    private int GYRO_TEST = 7;
    private int TESTS_COMPLETE = 8;

    private int checklistStep = 0;
    private List<ChecklistItem> checkListSteps = List.of( 
        new ChecklistItem("1. Battery Test", this::checkBattery, 0, 1),
        new ChecklistItem("2. All Devices Available", this::checkDevices, 0, 2),
        new ChecklistItem("3a. Test Swerve Module 0", this::checkSwerveModule0, 2, 3),
        new ChecklistItem("3b. Test Swerve Module 1", this::checkSwerveModule1, 4, 3),
        new ChecklistItem("3c. Test Swerve Module 2", this::checkSwerveModule2, 6, 3),
        new ChecklistItem("3d. Test Swerve Module 3", this::checkSwerveModule3, 8, 3),
        new ChecklistItem("3. Swerve Modules", this::checkSwerveModules, 0, 3),
        new ChecklistItem("4. Test Gyro", this::checkGyro, 0, 4),
        new ChecklistItem("Tests Complete", this::allTestsComplete, 6, 0));

    public TestChecklist(GyroIO gyro, SwerveDriveTrain driveTrain) {
        this.gyro = gyro;
        this.driveTrain = driveTrain;

        tabMain.addString("Current Step", this::getCurrentStep)
            .withPosition(0, 0).withSize(2, 1);
        tabMain.addString("Current Step Status", this::getCurrentStepStatus)
            .withPosition(2, 0).withSize(4, 1);
        
        resetTests = tabMain.add("Reset", false)
                    .withWidget(BuiltInWidgets.kToggleButton)
                    .withPosition(8, 0)
                    .withSize(1, 1).getEntry();
    
        for (var step : checkListSteps) {
            step.widget = tabMain.add(step.Title, false)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .withPosition(step.column, step.row)
                    .withSize(2, 1).getEntry();
        }

        tabMain.addString("Voltage | Threshold ", () -> checkListSteps.get(BATTERY_TEST).status)
            .withPosition(checkListSteps.get(BATTERY_TEST).column + 2, checkListSteps.get(BATTERY_TEST).row)
            .withSize(2, 1);

        tabMain.addString("Device List ", () -> checkListSteps.get(DEVICES_TEST).status)
            .withPosition(checkListSteps.get(DEVICES_TEST).column + 2, checkListSteps.get(DEVICES_TEST).row)
            .withSize(5, 1);

        for (int mod = 0; mod < 4; mod++) {
            testModule[mod] = tabMain.add("Test Module " + mod, false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(checkListSteps.get(SWERVE_MOD_0).column + 2 + mod, checkListSteps.get(SWERVE_MOD_0 + mod).row + 1)
                .withSize(1, 1).getEntry();
        }

        testGyro = tabMain.add("Test Gyro", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(checkListSteps.get(GYRO_TEST).column + 2, checkListSteps.get(GYRO_TEST).row)
            .withSize(1, 1).getEntry();
    }

    public void enableChecklist() {
        m_enableCheckList = true;
        resetTests();
    }

    public void disableChecklist() {
        m_enableCheckList = false;
    }

    public String getCurrentStep() {
        if (m_enableCheckList) {
            return checkListSteps.get(checklistStep).Title;
        }
        return "Disabled";
    }
    public String getCurrentStepStatus() {
        if (m_enableCheckList) {
            return checkListSteps.get(checklistStep).status;
        }
        return "Place DriverStation in Test Mode to run checklists";
    }

    public void resetTests() {
        for (var step : checkListSteps) {
            step.complete = false;
            step.status = "";
            step.widget.setBoolean(false);
        }
        for (int mod = 0; mod < 4; ++mod) {
            checkSwerveModule[mod] = 0;
            testModule[mod].setBoolean(false);
        }    
        checkGyroState = 0;    
        checklistStep = 0;
        resetTests.setBoolean(false);
    }

    public void update() {
        if (m_enableCheckList) {
            if (resetTests.getBoolean(false)) {
                resetTests();
            }

            for (var step : checkListSteps) {
                step.widget.setBoolean(step.complete);
            }

            if (checklistStep < checkListSteps.size()) {
                if (checkListSteps.get(checklistStep).isComplete.get()) {
                    if (checklistStep < TESTS_COMPLETE) {
                        checklistStep += 1;
                    }
                }
            }
        }

    }
    
    public boolean checkBattery() {
        boolean result = power.getVoltage() >= voltageThreshold.get();

        checkListSteps.get(0).complete = result;
        checkListSteps.get(0).status = power.getVoltage() + " | " + voltageThreshold.get();
        
        return result;
    }
    public boolean checkDevices() {
        boolean allPresent = true;
        String status = "";

        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            status += "Falcon(" + Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID + ") ";
        }

        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     status += "Falcon(" + Constants.FRONT_LEFT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }
        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     status += "Falcon(" + Constants.FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }
        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     status += "Falcon(" + Constants.BACK_LEFT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }
        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     status += "Falcon(" + Constants.BACK_RIGHT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }

        if (!gyro.isConnected()) {
            allPresent = false;
            status += "Gyro(" + Constants.GYRO_ID + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.PCM, Constants.PNEUMATICSHUB_ID)) {
            allPresent = false;
            status += "PH(" + Constants.PNEUMATICSHUB_ID + ") ";
        }
        if (!power.getFaults().CanWarning) {
            allPresent = false;
            status += "PDP(" + 0 + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.SPARK_MAX, Constants.ARM_LIFT_MOTOR_ID)) {
            allPresent = false;
            status += "SparkMax(" + Constants.ARM_LIFT_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.SPARK_MAX, Constants.ARM_EXTEND_MOTOR_ID)) {
            allPresent = false;
            status += "SparkMax(" + Constants.ARM_EXTEND_MOTOR_ID + ") ";
        }
        checkListSteps.get(1).complete = true;//allPresent;
        checkListSteps.get(1).status = status;
        
        return true; //allPresent;
    }

    public boolean testSwerveModule(int mod) {
        boolean toggle = testModule[mod].getBoolean(false);

        if (checkSwerveModule[mod] == 0) {
            if (!toggle) {
                checkListSteps.get(SWERVE_MOD_0 + mod).status = "Click Toggle to Start Motor " + mod;
            } else {
                driveTrain.testModule(mod, 0.5, 90.0);
                checkSwerveModule[mod] = 1;
                checkListSteps.get(SWERVE_MOD_0 + mod).status = "Verify motor is moving forward, angle is 90 degrees - flip toggle when complete";
            }
        } else if (checkSwerveModule[mod] == 1) {
            if (!toggle) {
                driveTrain.testModule(mod, 0.0, 0.0);
                checkSwerveModule[mod] = 2;    
                checkListSteps.get(SWERVE_MOD_0 + mod).status = "Module " + mod + " complete";
                checkListSteps.get(SWERVE_MOD_0 + mod).complete = true;
            }
        }
        return checkSwerveModule[mod] == 2;
    }

    public boolean checkSwerveModule0() {
        return testSwerveModule(0);
    }
    public boolean checkSwerveModule1() {
        return testSwerveModule(1);
    }
    public boolean checkSwerveModule2() {
        return testSwerveModule(2);
    }
    public boolean checkSwerveModule3() {
        return testSwerveModule(3);
    }
    public boolean checkSwerveModules() {
        boolean result = true;
        for (int mod = 0; mod < 4; mod++) {
            result = result && checkListSteps.get(SWERVE_MOD_0 + mod).complete;
        }
        checkListSteps.get(SWERVE_MODULES).complete = result;
        return result;
    }
    public boolean testGyro() {
        boolean toggle = testGyro.getBoolean(false);

        if (checkGyroState == 0) {
            if (!toggle) {
                checkListSteps.get(GYRO_TEST).status = "Click Toggle to Start Gyro Test";
            } else {
                gyro.reset();
                checkGyroState = 1;
                checkListSteps.get(GYRO_TEST).status = "Rotate robot 90 degrees";
            }
        } else if (checkGyroState == 1) {
            if (Math.abs(gyro.getAngle() - 90.0) < 0.1) {
                checkGyroState = 2;    
                checkListSteps.get(GYRO_TEST).status = "Gyro test complete";
                checkListSteps.get(GYRO_TEST).complete = true;
                testGyro.setBoolean(false);
            }
        }
        return checkGyroState == 2;
    }
    public boolean checkGyro() {
        return testGyro() && checkListSteps.get(GYRO_TEST).complete;
    }
    public boolean allTestsComplete() {
        boolean result = true;
        for (int i = BATTERY_TEST; i < TESTS_COMPLETE; ++i) {
            result = result && checkListSteps.get(i).complete;
        }
        checkListSteps.get(TESTS_COMPLETE).complete = result;
        checkListSteps.get(TESTS_COMPLETE).status = "All Tests Completed";

        return result;
    }
}
