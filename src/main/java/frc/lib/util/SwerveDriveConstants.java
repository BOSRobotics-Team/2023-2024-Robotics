package frc.lib.util;

import edu.wpi.first.math.util.Units;

public final class SwerveDriveConstants {

  /** diameter in meters */
  public static final double wheelDiameter_MK4 = Units.inchesToMeters(4.0);

  /* Module Gear Ratios SDS MK4 */
  /** SDS MK4 - 8.14 : 1 */
  public static final double driveGearRatio_MK4_L1 = 8.14;

  /** SDS MK4 - 6.75 : 1 */
  public static final double driveGearRatio_MK4_L2 = 6.75;

  /** SDS MK4 - 6.12 : 1 */
  public static final double driveGearRatio_MK4_L3 = 6.12;

  /** SDS MK4 - 5.14 : 1 */
  public static final double driveGearRatio_MK4_L4 = 5.14;

  /** SDS MK4 - 12.8 : 1 */
  public static final double angleGearRatio_MK4 = 12.8;

  public static final boolean driveMotorInvert_MK4 = false;

  public static final boolean angleMotorInvert_MK4 = false;

  /* Module Gear Ratios SDS MK4i */
  /** diameter in meters */
  public static final double wheelDiameter_MK4i = Units.inchesToMeters(4.0);

  /** SDS MK4i - 8.14 : 1 */
  public static final double driveGearRatio_MK4i_L1 = 8.14;

  /** SDS MK4i - 6.75 : 1 */
  public static final double driveGearRatio_MK4i_L2 = 6.75;

  /** SDS MK4i - 6.12 : 1 */
  public static final double driveGearRatio_MK4i_L3 = 6.12;

  /** SDS MK4i - (150 / 7) : 1 */
  public static final double angleGearRatio_MK4i = 150.0 / 7.0;

  public static final boolean driveMotorInvert_MK4i = false;

  public static final boolean angleMotorInvert_MK4i = true;
}
