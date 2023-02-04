package frc.robot.subsystems.arm;

public final class ArmConstants {
    // PID coefficients
    public static final double armLiftKP = 0.1;
    public static final double armLiftKI = 0.0;
    public static final double armLiftKD = 0.0;
    public static final double armLiftKIZ = 0.0;
    public static final double armLiftKFF = 0.000156; 
    public static final double armLiftMaxOutput = 0.15; 
    public static final double armLiftMinOutput = -0.15; 
    public static final double armLiftMaxRPM = 5700.0;
    public static final double armLiftMinPosition = 0.0;
    public static final double armLiftMaxPosition = 40.0;

    public static final double armExtendKP = 0.2; 
    public static final double armExtendKI = 0.0;
    public static final double armExtendKD = 0.0;
    public static final double armExtendKIZ = 0.0;
    public static final double armExtendKFF = 0.000156; 
    public static final double armExtendMaxOutput = 0.5; 
    public static final double armExtendMinOutput = -0.5; 
    public static final double armExtendMaxRPM = 5700.0;
    public static final double armExtendMinPosition = 0.0;
    public static final double armExtendMaxPosition = 400.0;

    // Smart Motion Coefficients
    public static final double armLiftMaxVel = 2000.0; 
    public static final double armLiftMinVel = 50.0; 
    public static final double armLiftMaxAcc = 1500.0;
    public static final double armLiftAllowedErr = 0.0;
    public static final double armLiftGearRatio = 0.005208333333; // 192:1
    public static final double armLiftMetersPerRotation = 0.1;
    public static final double armLiftMinHeight = 0.0;
    public static final double armLiftMaxHeight = 2.0;

    public static final double armExtendMaxVel = 2000.0; 
    public static final double armExtendMinVel = 50.0; 
    public static final double armExtendMaxAcc = 1500.0; 
    public static final double armExtendAllowedErr = 0.0;
    public static final double armExtendGearRatio = 0.0125; //80:1
    public static final double armExtendMetersPerRotation = 0.1;
    public static final double armExtendMinLength = 0.0;
    public static final double armExtendMaxLength = 2.0;
}
