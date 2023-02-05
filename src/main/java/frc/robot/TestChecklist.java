package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.CANDeviceFinder;
import frc.lib.util.DashboardNumber;
import frc.lib.util.DriveGyro;
import frc.lib.util.CANDeviceId.CANDeviceType;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;

public class TestChecklist {

    private final ShuffleboardTab tabMain = Shuffleboard.getTab("Checklist");

    private final PowerDistribution power = new PowerDistribution();
    private final CANDeviceFinder canFinder = new CANDeviceFinder();

    private final DriveGyro gyro;
    private final SwerveDriveTrain driveTrain;

    private final DashboardNumber voltageThreshold = new DashboardNumber("Checklist/VoltageThreshold", 12.0);

    private String checkBatteryStr = "";
    private String checkDevicesStr = "";

    private int checkSwerveModule[] = {0, 0, 0, 0};
    private GenericEntry testModule[] = {null, null, null, null};

    private int checkGyroState = 0;
    private GenericEntry testGyro;


    public TestChecklist(DriveGyro gyro, SwerveDriveTrain driveTrain) {
        this.gyro = gyro;
        this.driveTrain = driveTrain;

        // tabMain.addNumber("Gyroscope Angle", this::getRotationDegrees);
        // tabMain.addBoolean("X-Stance On?", this::isXstance);
        tabMain.addBoolean("1. Battery Test", this::checkBattery)
            .withPosition(0, 0)
            .withSize(2, 1);
        tabMain.addString("Voltage | Threshold ", () -> this.checkBatteryStr)
            .withPosition(2, 0)
            .withSize(2, 1);

        tabMain.addBoolean("2. All Devices Available", this::checkDevices)
            .withPosition(0, 1)
            .withSize(2, 1);
        tabMain.addString("Device List ", () -> this.checkDevicesStr)
            .withPosition(2, 1)
            .withSize(5, 1);

        tabMain.addBoolean("3. Test Swerve Modules", this::checkSwerveModules)
            .withPosition(0, 2)
            .withSize(2, 1);
        for (int mod = 0; mod < 4; mod++) {
            testModule[mod] = tabMain.add("Test Swerve Module " + mod, checkSwerveModule[mod] == 1)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(2 + mod, 2)
                .withSize(1, 1).getEntry();
        }

        tabMain.addBoolean("4. Test Gyro", this::checkGyro)
            .withPosition(0, 3)
            .withSize(2, 1);
        testGyro = tabMain.add("Test Gyro", checkGyroState == 1)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(2, 3)
            .withSize(1, 1).getEntry();
}

    public boolean checkBattery() {
        checkBatteryStr = power.getVoltage() + " | " + voltageThreshold.get();
        return power.getVoltage() >= voltageThreshold.get();
    }
    public boolean checkDevices() {
        boolean allPresent = true;
        this.checkDevicesStr = "";

        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.FRONT_LEFT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.FRONT_RIGHT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.BACK_LEFT_MODULE_ANGLE_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "Falcon(" + Constants.BACK_RIGHT_MODULE_ANGLE_MOTOR_ID + ") ";
        }

        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_LEFT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     this.checkDevicesStr += "Falcon(" + Constants.FRONT_LEFT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }
        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     this.checkDevicesStr += "Falcon(" + Constants.FRONT_RIGHT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }
        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_LEFT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     this.checkDevicesStr += "Falcon(" + Constants.BACK_LEFT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }
        // if (!canFinder.isDevicePresent(CANDeviceType.TALON, Constants.BACK_RIGHT_MODULE_ANGLE_ENCODER_ID)) {
        //     allPresent = false;
        //     this.checkDevicesStr += "Falcon(" + Constants.BACK_RIGHT_MODULE_ANGLE_ENCODER_ID + ") ";
        // }

        if (!gyro.isConnected()) {
            allPresent = false;
            this.checkDevicesStr += "Gyro(" + Constants.GYRO_ID + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.PCM, Constants.PNEUMATICSHUB_ID)) {
            allPresent = false;
            this.checkDevicesStr += "PH(" + Constants.PNEUMATICSHUB_ID + ") ";
        }
        if (!power.getFaults().CanWarning) {
            allPresent = false;
            this.checkDevicesStr += "PDP(" + 0 + ") ";
        }

        if (!canFinder.isDevicePresent(CANDeviceType.SPARK_MAX, Constants.ARM_LIFT_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "SparkMax(" + Constants.ARM_LIFT_MOTOR_ID + ") ";
        }
        if (!canFinder.isDevicePresent(CANDeviceType.SPARK_MAX, Constants.ARM_EXTEND_MOTOR_ID)) {
            allPresent = false;
            this.checkDevicesStr += "SparkMax(" + Constants.ARM_EXTEND_MOTOR_ID + ") ";
        }

        return allPresent;
    }

    public void update() {
        for (int mod = 0; mod < 4; mod++) {
            if ((checkSwerveModule[mod] == 0) && testModule[mod].getBoolean(false)) {
                driveTrain.testModule(mod, 0.5, 90.0);
                checkSwerveModule[mod] = 1;
            } else if ((checkSwerveModule[mod] == 1) && !testModule[mod].getBoolean(false)) {
                driveTrain.testModule(mod, 0.0, 0.0);
                checkSwerveModule[mod] = 2;
            }
        }
        if ((checkGyroState == 0) && testGyro.getBoolean(false)) {
            gyro.reset();
            checkGyroState = 1;
        } else if ((checkGyroState == 1) && (gyro.getHeadingDegrees() >= 90.0)) {
            checkGyroState = 2;
        }

    }
    public boolean checkSwerveModules() {
        return (checkSwerveModule[0] == 2) && (checkSwerveModule[1] == 2) && (checkSwerveModule[2] == 2) && (checkSwerveModule[3] == 2);
    }
    public boolean checkGyro() {
        return (checkGyroState == 2);
    }
}
