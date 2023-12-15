package frc.robot.testsystem;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;

public class TestChecklist {
    
    private RobotContainer m_robot = RobotContainer.GetInstance();
    private Boolean m_enableTestChecklist = false; // Decides whether tests can be Run

    public TestableSubsytem[] m_subsytems; // List of available Subsystems
    public TestChecklist(TestableSubsytem... subsytems) { m_subsytems = subsytems; }

    /* Test Runner Implementation */
    public void initialize() {
        m_enableTestChecklist = true;
    }
    public void periodic() {
        if (m_enableTestChecklist) { }
    }
    public void exit() {
        m_enableTestChecklist = false;
    }

    /* Custom Teleop Mode Implementation */ 
    public void doTeleop() {

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
