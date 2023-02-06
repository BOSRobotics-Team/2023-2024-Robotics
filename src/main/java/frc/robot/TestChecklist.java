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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;

public class TestChecklist {
    private class ChecklistItem {
        public final String Title;
        public final Supplier<Boolean> runTest;
        public final int column;
        public final int row;
        
        public GenericEntry widget;
        public Boolean complete;
        public String status;

        public ChecklistItem( String title, Supplier<Boolean> test, int col, int row) {
            this.Title = title;
            this.runTest = test;
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
    private final Arm arm;
    

    private final DashboardNumber voltageThreshold = new DashboardNumber("Checklist/VoltageThreshold", 12.0);

    private GenericEntry resetTests;

    private int checkSwerveModule[] = {0, 0, 0, 0};
    private GenericEntry testModule[] = {null, null, null, null};

    private int checkGyroState[] = {0, 0};
    private GenericEntry testGyro[] = {null, null};

    private int checkLimitSwitchState[] = {0, 0};


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
    
    private int TESTS_COMPLETE = 13;

    private int checklistStep = 0;
    private List<ChecklistItem> checkListSteps = List.of( 
        new ChecklistItem("1. Battery Test", this::checkBattery, 0, 1),
        new ChecklistItem("2. All Devices Available", this::checkDevices, 0, 2),
        new ChecklistItem("3a. Swerve Module 0", this::checkSwerveModule0, 2, 3),
        new ChecklistItem("3b. Swerve Module 1", this::checkSwerveModule1, 4, 3),
        new ChecklistItem("3c. Swerve Module 2", this::checkSwerveModule2, 6, 3),
        new ChecklistItem("3d. Swerve Module 3", this::checkSwerveModule3, 8, 3),
        new ChecklistItem("3. Swerve Modules", this::checkSwerveModules, 0, 3),
        new ChecklistItem("4a. Test Yaw", this::checkGyroYaw, 2, 4),
        new ChecklistItem("4b. Test Pitch", this::checkGyroPitch, 5, 4),
        new ChecklistItem("4. Test Gyro", this::checkGyro, 0, 4),
        new ChecklistItem("5a. ArmExtend Switch", this::checkArmExtendSwitch, 2, 5),
        new ChecklistItem("5b. ArmLift Switch", this::checkArmLiftSwitch, 4, 5),
        new ChecklistItem("5. Limit Switches", this::checkLimitSwitches, 0, 5),
        new ChecklistItem("Tests Complete", this::allTestsComplete, 6, 0));

    public TestChecklist(GyroIO gyro, SwerveDriveTrain driveTrain, Arm arm) {
        this.gyro = gyro;
        this.driveTrain = driveTrain;
        this.arm = arm;

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
                .withPosition(10 + mod, checkListSteps.get(SWERVE_MOD_0 + mod).row)
                .withSize(1, 1).getEntry();
        }

        testGyro[0] = tabMain.add("Test Yaw", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(checkListSteps.get(GYRO_YAWTEST).column + 2, checkListSteps.get(GYRO_YAWTEST).row)
            .withSize(1, 1).getEntry();
        testGyro[1] = tabMain.add("Test Pitch", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(checkListSteps.get(GYRO_PITCHTEST).column + 2, checkListSteps.get(GYRO_PITCHTEST).row)
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
        for (int i = 0; i < 2; ++i) {
            checkGyroState[i] = 0; 
            testGyro[i].setBoolean(false);
        }
        for (int i = 0; i < 2; ++i) {
            checkLimitSwitchState[i] = 0;
        }
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
                if (checkListSteps.get(checklistStep).runTest.get()) {
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
        checkListSteps.get(1).complete = allPresent;
        checkListSteps.get(1).status = status;
        
        return allPresent;
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
        checkListSteps.get(SWERVE_MODULES).status = "Module Tests complete";

        return result;
    }
    
    public boolean checkGyroYaw() {
        boolean toggle = testGyro[0].getBoolean(false);
        if (checkGyroState[0] == 0) {
            if (!toggle) {
                checkListSteps.get(GYRO_YAWTEST).status = "Click Toggle to Start Gyro Yaw Test";
            } else {
                gyro.reset();
                checkGyroState[0] = 1;
                checkListSteps.get(GYRO_YAWTEST).status = "Rotate robot 90 degrees";
            }
        } else if (checkGyroState[0] == 1) {
            if (Math.abs(gyro.getAngle() - 90.0) < 0.1) {
                checkGyroState[0] = 2;    
                checkListSteps.get(GYRO_YAWTEST).status = "Gyro test complete";
                checkListSteps.get(GYRO_YAWTEST).complete = true;
                testGyro[0].setBoolean(false);
            }
        }
        return checkGyroState[0] == 2;
    }
    public boolean checkGyroPitch() {
        boolean toggle = testGyro[1].getBoolean(false);
        if (checkGyroState[1] == 0) {
            if (!toggle) {
                checkListSteps.get(GYRO_PITCHTEST).status = "Click Toggle to Start Gyro Pitch Test";
            } else {
                gyro.reset();
                checkGyroState[1] = 1;
                checkListSteps.get(GYRO_PITCHTEST).status = "Lift robot 30 degrees";
            }
        } else if (checkGyroState[1] == 1) {
            if (Math.abs(gyro.getPitch() - 30.0) < 0.1) {
                checkGyroState[1] = 2;    
                checkListSteps.get(GYRO_PITCHTEST).status = "Gyro test complete";
                checkListSteps.get(GYRO_PITCHTEST).complete = true;
                testGyro[1].setBoolean(false);
            }
        }
        return checkGyroState[1] == 2;
    }
    public boolean checkGyro() {
        boolean result = 
            checkListSteps.get(GYRO_YAWTEST).complete &&
            checkListSteps.get(GYRO_PITCHTEST).complete;

        checkListSteps.get(GYRO_TEST).complete = result;
        checkListSteps.get(GYRO_TEST).status = "Gyro test complete";
        return result;
    }
    public boolean checkArmExtendSwitch() {
        if (checkLimitSwitchState[0] == 0) {
            if (!arm.isArmExtendMinLimitSwitch()) {
                checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST).status = "Click Arm Extend Limit Switch";
            } else {
                checkLimitSwitchState[0] = 1;
                checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST).status = "Release Arm Extend Limit Switch";
            }
        } else if ((checkLimitSwitchState[0] == 1) && !arm.isArmExtendMinLimitSwitch()) {
            checkLimitSwitchState[0] = 2;
            checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST).status = "Arm Extend Limit Switch complete";
            checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST).complete = true;
        }
        return checkLimitSwitchState[0] == 2;
    }
    public boolean checkArmLiftSwitch() {
        if (checkLimitSwitchState[1] == 0) {
            if (!arm.isArmLiftMinLimitSwitch()) {
                checkListSteps.get(LIMITSWITCH_ARMLIFTTEST).status = "Click Arm Lift Limit Switch";
            } else {
                checkLimitSwitchState[1] = 1;
                checkListSteps.get(LIMITSWITCH_ARMLIFTTEST).status = "Release Arm Lift Limit Switch";
            }
        } else if ((checkLimitSwitchState[1] == 1) && !arm.isArmExtendMinLimitSwitch()) {
            checkLimitSwitchState[1] = 2;
            checkListSteps.get(LIMITSWITCH_ARMLIFTTEST).status = "Arm Lift Limit Switch complete";
            checkListSteps.get(LIMITSWITCH_ARMLIFTTEST).complete = true;
        }
        return checkLimitSwitchState[1] == 2;
    }
    public boolean checkLimitSwitches() {
        boolean result = 
            checkListSteps.get(LIMITSWITCH_ARMEXTENDTEST).complete &&
            checkListSteps.get(LIMITSWITCH_ARMLIFTTEST).complete;

        checkListSteps.get(LIMITSWITCH_TEST).complete = result;
        checkListSteps.get(LIMITSWITCH_TEST).status = "Arm Limit Switches complete";
        return result;
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
