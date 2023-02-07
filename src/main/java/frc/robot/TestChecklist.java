package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.lib.util.CANDeviceFinder;
import frc.lib.util.DashboardNumber;
import frc.lib.util.CANDeviceId.CANDeviceType;

public class TestChecklist {
    private class ChecklistItem {
        public final String Title;
        public final Supplier<Boolean> runTest;
        public final int column;
        public final int row;
        
        // public GenericEntry widgetEntry;
        public SimpleWidget widget;
        public String status;
        public int state;

        public ChecklistItem( String title, Supplier<Boolean> test, int col, int row) {
            this.Title = title;
            this.runTest = test;
            this.column = col;
            this.row = row;
            this.widget = null;
            this.status = "";
            this.state = 0;
        }
        public void initTab(ShuffleboardTab tab) {
            this.widget = tab.add(this.Title, false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("Color when false", "grey"))
                .withPosition(this.column, this.row)
                .withSize(2, 1);                

            // if (this.state == 0) {
            //     this.toggle = tabMain.add(this.Title + " Test", false)
            //     .withWidget(BuiltInWidgets.kToggleSwitch)
            //     .withPosition(this.column == 0 ? 2 : this.column + 1, this.row)
            //     .withSize(1, 1).getEntry();
            // }
        }
        public void reset() {
            this.state = 0;
            this.status = "";
            setComplete(false);
            this.widget.withProperties(Map.of("Color when false", "grey"));
        }
        public boolean setComplete(boolean result) {
            setCurrentStep();
            this.widget.getEntry().setBoolean(result);
            return result;
        }
        public boolean isComplete() {
            return this.widget.getEntry().getBoolean(false);
        }
        public void setCurrentStep() {
            this.widget.withProperties(Map.of("Color when false", "red"));
        }
    }

    private final ShuffleboardTab tabMain = Shuffleboard.getTab("Checklist");
    private final CANDeviceFinder canFinder = new CANDeviceFinder();
    private final RobotContainer robot;

    private final DashboardNumber voltageThreshold = new DashboardNumber("Checklist/VoltageThreshold", 12.0);

    private GenericEntry resetTestsWidget;
    private GenericEntry doStepWidget;

    private boolean m_enableCheckList = false;

    private int BATTERY_TEST = 0;
    private int DEVICES_TEST = 1;
    private int SWERVE_MOD_0 = 2;
    // private int SWERVE_MOD_1 = 3;
    // private int SWERVE_MOD_2 = 4;
    // private int SWERVE_MOD_3 = 5;
    private int SWERVE_MODULES = 6;
    private int GYRO_YAWTEST = 7;
    private int GYRO_PITCHTEST = 8;
    private int GYRO_TEST = 9;
    private int LIMITSWITCH_ARMEXTENDTEST = 10;
    private int LIMITSWITCH_ARMLIFTTEST = 11;
    private int LIMITSWITCH_TEST = 12;
    private int ARM_CALIBRATE = 13;
    private int ARM_MAXLIFTTEST = 14;
    private int ARM_MAXEXTENDTEST = 15;
    private int ARM_TOZERO = 16;
    private int ARM_TEST = 17;
    private int TESTS_COMPLETE = 18;

    private int checklistStep = 0;
    private List<ChecklistItem> checkListSteps = List.of( 
        new ChecklistItem("1. Battery Test", this::checkBattery, 0, 1),
        new ChecklistItem("2. CAN Devices", this::checkDevices, 0, 2),
        new ChecklistItem("3a. Module 0", this::checkSwerveModule0, 2, 3),
        new ChecklistItem("3b. Module 1", this::checkSwerveModule1, 4, 3),
        new ChecklistItem("3c. Module 2", this::checkSwerveModule2, 6, 3),
        new ChecklistItem("3d. Module 3", this::checkSwerveModule3, 8, 3),
        new ChecklistItem("3. Swerve Module Test", this::checkSwerveModules, 0, 3),
        new ChecklistItem("4a. Yaw", this::checkGyroYaw, 2, 4),
        new ChecklistItem("4b. Pitch", this::checkGyroPitch, 4, 4),
        new ChecklistItem("4. Gyro Test", this::checkGyro, 0, 4),
        new ChecklistItem("5a. Extend", this::checkArmExtendSwitch, 2, 5),
        new ChecklistItem("5b. Lift", this::checkArmLiftSwitch, 4, 5),
        new ChecklistItem("5. Limit Switches", this::checkLimitSwitches, 0, 5),
        new ChecklistItem("6a. Calibrate", this::checkArmCalibrate, 2, 6),
        new ChecklistItem("6b. Lift MaxHeight", this::checkArmLiftMaxHeight, 4, 6),
        new ChecklistItem("6c. Extend MaxLength", this::checkArmExtendMaxLength, 6, 6),
        new ChecklistItem("6d. Zero Arm", this::checkArmToZero, 8, 6),
        new ChecklistItem("6. Arm Tests", this::checkArm, 0, 6),
        new ChecklistItem("Tests Complete", this::allTestsComplete, 8, 0));

    public TestChecklist(RobotContainer container) {
        this.robot = container;

        tabMain.addString("Current Step", this::getCurrentStep)
            .withPosition(0, 0).withSize(2, 1);
        tabMain.addString("Current Step Status", this::getCurrentStepStatus)
            .withPosition(3, 0).withSize(5, 1);
        
        resetTestsWidget = tabMain.add("Reset", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(10, 0)
            .withSize(1, 1).getEntry();
    
        doStepWidget = tabMain.add("Step", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 0)
            .withSize(1, 1).getEntry();

        for (var step : checkListSteps) {
            step.initTab(tabMain);
        }

        tabMain.addString("Voltage | Threshold ", () -> checkListSteps.get(BATTERY_TEST).status)
            .withPosition(checkListSteps.get(BATTERY_TEST).column + 2, checkListSteps.get(BATTERY_TEST).row)
            .withSize(2, 1);

        tabMain.addString("Device List ", () -> checkListSteps.get(DEVICES_TEST).status)
            .withPosition(checkListSteps.get(DEVICES_TEST).column + 2, checkListSteps.get(DEVICES_TEST).row)
            .withSize(6, 1);
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

    public boolean getDoStep() {
        return doStepWidget.getBoolean(false);
    }
    public boolean resetDoStep() {
        return doStepWidget.setBoolean(false);
    }

    public void resetTests() {
        for (var step : checkListSteps) {
            step.reset();
        }
        resetDoStep();

        checklistStep = 0;
        resetTestsWidget.setBoolean(false);
    }

    public void update() {
        if (m_enableCheckList) {
            if (resetTestsWidget.getBoolean(false)) {
                resetTests();
            }

            if (checklistStep < checkListSteps.size()) {
                if (checkListSteps.get(checklistStep).runTest.get()) {
                    if (checklistStep < TESTS_COMPLETE) {
                        checklistStep += 1;
                    }
                }
            }
        }

    }
    
    public boolean checkBattery() {
        ChecklistItem item = checkListSteps.get(BATTERY_TEST);
        item.status = robot.power.getVoltage() + " | " + voltageThreshold.get();

        return item.setComplete(robot.power.getVoltage() >= voltageThreshold.get());
    }
    public boolean checkDevices() {
        ChecklistItem item = checkListSteps.get(DEVICES_TEST);
        boolean allPresent = true;

        item.status = "";
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            item.status += "Falcon(" + Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID + ") ";
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

        if (!robot.gyro.isConnected()) {
            allPresent = false;
            item.status += "Gyro(" + Constants.GYRO_ID + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.PCM, Constants.PNEUMATICSHUB_ID)) {
            allPresent = false;
            item.status += "PH(" + Constants.PNEUMATICSHUB_ID + ") ";
        }
        if (!robot.power.getFaults().CanWarning) {
            allPresent = false;
            item.status += "PDP(" + 0 + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.SPARK_MAX, Constants.ARM_LIFT_MOTOR_ID)) {
            allPresent = false;
            item.status += "SparkMax(" + Constants.ARM_LIFT_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.SPARK_MAX, Constants.ARM_EXTEND_MOTOR_ID)) {
            allPresent = false;
            item.status += "SparkMax(" + Constants.ARM_EXTEND_MOTOR_ID + ") ";
        }
        return item.setComplete(allPresent);
    }

    public boolean testSwerveModule(int mod) {
        ChecklistItem item = checkListSteps.get(SWERVE_MOD_0 + mod);
        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click Toggle to Start Motor " + mod;
            } else {
                robot.driveTrain.testModule(mod, 0.5, 90.0);
                item.state = 1;
                item.status = "Verify motor is moving forward, angle is 90 degrees - flip toggle when complete";
            }
        } else if (item.state == 1) {
            if (!getDoStep()) {
                robot.driveTrain.testModule(mod, 0.0, 0.0);
                item.state = 2;    
                item.status = "Module " + mod + " complete";
                item.setComplete(true);
            }
        }
        return item.isComplete();
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
        ChecklistItem item = checkListSteps.get(SWERVE_MODULES);
        boolean result = true;
        for (int mod = 0; mod < 4; mod++) {
            result = result && checkListSteps.get(SWERVE_MOD_0 + mod).isComplete();
        }
        item.status = "Module Tests complete";
        return item.setComplete(result);
    }
    
    public boolean checkGyroYaw() {
        ChecklistItem item = checkListSteps.get(GYRO_YAWTEST);
        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click DoStep to start Yaw Test";
            } else {
                robot.gyro.reset();
                item.state = 1;
                item.status = "Rotate robot 90 degrees";
            }
        } else if (item.state == 1) {
            if (Math.abs(robot.gyro.getAngle() - 90.0) < 0.1) {
                item.state = 2;
                item.status = "Gyro test complete";
                item.setComplete(true);
                resetDoStep();
            }
        }
        return item.isComplete();
    }
    public boolean checkGyroPitch() {
        ChecklistItem item = checkListSteps.get(GYRO_PITCHTEST);
        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click Toggle to Start Gyro Pitch Test";
            } else {
                robot.gyro.reset();
                item.state = 1;
                item.status = "Lift robot 30 degrees";
            }
        } else if (item.state == 1) {
            if (Math.abs(robot.gyro.getPitch() - 30.0) < 0.1) {
                item.state = 2;    
                item.status = "Gyro test complete";
                item.setComplete(true);
                resetDoStep();
            }
        }
        return item.isComplete();
    }
    public boolean checkGyro() {
        ChecklistItem item = checkListSteps.get(GYRO_TEST);
        boolean result = 
            checkListSteps.get(GYRO_YAWTEST).isComplete() &&
            checkListSteps.get(GYRO_PITCHTEST).isComplete();
        item.status = "Gyro test complete";

        return item.setComplete(result);
    }
    public boolean checkArmExtendSwitch() {
        ChecklistItem item = checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST);
        if (item.state == 0) {
            if (!robot.arm.isArmExtendMinLimitSwitch()) {
                item.status = "Click Arm Extend Limit Switch";
            } else {
                item.state = 1;
                item.status = "Release Arm Extend Limit Switch";
            }
        } else if ((item.state == 1) && !robot.arm.isArmExtendMinLimitSwitch()) {
            item.state = 2;
            item.status = "Arm Extend Limit Switch complete";
            item.setComplete(true);
        }
        return item.isComplete();
    }
    public boolean checkArmLiftSwitch() {
        ChecklistItem item = checkListSteps.get(LIMITSWITCH_ARMLIFTTEST);
        if (item.state == 0) {
            if (!robot.arm.isArmLiftMinLimitSwitch()) {
                item.status = "Click Arm Lift Limit Switch";
            } else {
                item.state = 1;
                item.status = "Release Arm Lift Limit Switch";
            }
        } else if ((item.state == 1) && !robot.arm.isArmExtendMinLimitSwitch()) {
            item.state = 2;
            item.status = "Arm Lift Limit Switch complete";
            item.setComplete(true);
        }
        return item.isComplete();
    }
    public boolean checkLimitSwitches() {
        ChecklistItem item = checkListSteps.get(LIMITSWITCH_TEST);
        boolean result = 
            checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST).isComplete() &&
            checkListSteps.get(LIMITSWITCH_ARMLIFTTEST).isComplete();
        item.status = "Arm Limit Switches complete";

        return item.setComplete(result);
    }
    public boolean checkArmCalibrate() {
        ChecklistItem item = checkListSteps.get(ARM_CALIBRATE);
        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click Toggle to Start Arm Calibration";
            } else {
                robot.arm.resetArm();
                item.state = 1;
                item.status = "Calibrating Arm";
            }
        } else if (item.state == 1) {
            if (!robot.arm.isResetting()) {
                item.state = 2;    
                item.status = "Arm Calibrate complete";
                item.setComplete(true);
                resetDoStep();
            }
        }
        return item.isComplete();
    }
    public boolean checkArmLiftMaxHeight() {
        ChecklistItem item = checkListSteps.get(ARM_MAXLIFTTEST);
        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click Toggle to Raise Arm to Max Height";
            } else {
                robot.arm.raiseArm(1.0);
                item.state = 1;
                item.status = "Raising Arm";
            }
        } else if ((item.state == 1) && robot.arm.isArmRaised()) {
            item.state = 2;
            item.status = "Arm Raised to Max Height - Measure";
            item.setComplete(true);
            resetDoStep();
        }
        return item.isComplete();
    }
    public boolean checkArmExtendMaxLength() {
        ChecklistItem item = checkListSteps.get(ARM_MAXEXTENDTEST);
        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click Toggle to Extend Arm to Max Length";
            } else {
                robot.arm.extendArm(1.0);
                item.state = 1;
                item.status = "Extending Arm";
            }
        } else if ((item.state == 1) && robot.arm.isArmExtended()) {
            item.state = 2;
            item.status = "Arm Extended to Max Length - Measure";
            item.setComplete(true);
            resetDoStep();
        }
        return item.isComplete();
    }
    public boolean checkArmToZero() {
        ChecklistItem item = checkListSteps.get(ARM_TOZERO);

        if (item.state == 0) {
            if (!getDoStep()) {
                item.status = "Click Toggle to reset Arm to Zero";
            } else {
                robot.arm.extendArm(0.0);
                item.state = 1;
                item.status = "Retracting Arm";
            }
        } else if ((item.state == 1) && robot.arm.isArmExtended()) {
            item.state = 2;
            robot.arm.raiseArm(0.0);
            item.status = "Lowering Arm";
        } else if ((item.state == 2) && robot.arm.isArmRaised()) {
            item.state = 3;
            item.status = "Arm Set to Zero Position";
            item.setComplete(true);
            resetDoStep();
        }
        return item.isComplete();
    }

    public boolean checkArm() {
        ChecklistItem item = checkListSteps.get(ARM_TEST);
        boolean result = 
            checkListSteps.get(ARM_MAXLIFTTEST).isComplete() &&
            checkListSteps.get(ARM_MAXEXTENDTEST).isComplete() &&
            checkListSteps.get(ARM_TOZERO).isComplete();
        item.status = "Arm Tests complete";

        return item.setComplete(result);
    }

    public boolean allTestsComplete() {
        ChecklistItem item = checkListSteps.get(TESTS_COMPLETE);
        boolean result = true;
        for (int i = BATTERY_TEST; i < TESTS_COMPLETE; ++i) {
            result = result && checkListSteps.get(i).isComplete();
        }
        item.status = "All Tests Completed";

        return item.setComplete(result);
    }
}
