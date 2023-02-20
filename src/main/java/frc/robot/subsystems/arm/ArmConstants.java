package frc.robot.subsystems.arm;

public final class ArmConstants {
  // PID coefficients
  public static final double armLiftKP = 0.1;
  public static final double armLiftKI = 0.0;
  public static final double armLiftKD = 0.0;
  public static final double armLiftKIZ = 0.0;
  public static final double armLiftKFF = 0.000156;
  public static final double armLiftGearRatio = 768.0; // 1:768
  public static final double armLiftMaxOutput = 0.4;
  public static final double armLiftMinOutput = -0.4;
  public static final double armLiftResetOutput = -0.1;
  public static final double armLiftMaxRPM = 5700.0;
  public static final double armLiftMinPosition = 0.0;
  public static final double armLiftMaxPosition = 240.0; // 0.3125 * armLiftGearRatio
  public static final double armLiftMoveThreshold = 1.0;

  public static final double armExtendKP = 0.2;
  public static final double armExtendKI = 0.0;
  public static final double armExtendKD = 0.0;
  public static final double armExtendKIZ = 0.0;
  public static final double armExtendKFF = 0.000156;
  public static final double armExtendGearRatio = 48.0; // 1:48
  public static final double armExtendMaxOutput = 0.5;
  public static final double armExtendMinOutput = -0.5;
  public static final double armExtendResetOutput = -0.1;
  public static final double armExtendMaxRPM = 5700.0;
  public static final double armExtendMinPosition = 0.0;
  public static final double armExtendMaxPosition = 300.0; // 500.0;    6.25 * armExtendGearRatio;
  public static final double armExtendMoveThreshold = 1.0;

  public static final double armLiftClawSafetyHeight = 32.0; // 0.04166666667 * armLiftGearRatio
  public static final double armLiftExtendSafetyHeight =
      45.0; // 75.0; // 0.9375 * armExtendGearRatio
  public static final double armExtendSafetyLength = 21.0; // 35.0; // 0.4375 * armExtendGearRatio

  public static final double[][] armLiftProfile = {
    {5.0, 21.0}, // 35.0 },
    {10.0, 30.0}, // 50.0 },
    {15.0, 36.0}, // 60,0 },
    {20.0, 42.0}, // 70.0 },
    {25.0, 45.0}, // 75.0 },
    {30.0, 108.0}, // 180.0 },
    {50.0, 180.0}, // 300.0 },
    {60.0, 216.0}, // 360.0 },
    {70.0, 270.0}, // 450.0 },
    {80.0, 330.0}
  }; // 550.0 }};

  public static final double[][] armPositionCone = {
    {0.0, 0.0},
    {72.0, 30.0}, // 50.0 },
    {200.0, 48.0}, // 80,0 },
    {228.0, 297.0}
  }; // 495.0 }};
  public static final double[][] armPositionCube = {
    {0.0, 0.0},
    {56.0, 30.0}, // 50.0 },
    {184.0, 48.0}, // 80,0 },
    {212.0, 297.0}
  }; // 495.0 }};
}
