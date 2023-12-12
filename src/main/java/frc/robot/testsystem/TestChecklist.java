package frc.robot.testsystem;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrainConstants;



public class TestChecklist {
    
    private RobotContainer m_robot = RobotContainer.GetInstance();
    public TestableSubsytem[] m_subsytems;
    private Boolean m_enableTestChecklist = false;

    public TestChecklist(TestableSubsytem... subsytems) {
        m_subsytems = subsytems;
    }

    public void initialize() {
        m_enableTestChecklist = true;
    }
    public void periodic() {
        if (m_enableTestChecklist) {
            
        }
    }
    public void exit() {
        m_enableTestChecklist = false;
    }

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
