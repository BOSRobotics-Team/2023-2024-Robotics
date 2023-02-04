package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Gets a value from dashboard in active mode, returns default if not or value in dashboard.
 */
public class DashboardNumber {
  public static boolean isActiveMode = false;

  private static final String TABLE_KEY = "DashboardNumbers/";

  private String key = "";
  private double defaultValue = 0.0;
  private double lastHasChangedValue = defaultValue;

  /**
   * Create a new DashboardNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public DashboardNumber(String dashboardKey) {
    this.key = TABLE_KEY + dashboardKey;
  }

  /**
   * Create a new DashboardNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public DashboardNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefaultValue(defaultValue);
  }

  /**
   * Set globale active mode on or off
   *
   * @param active active mode on or off
   */
  public static void setActiveMode(boolean active) {
    isActiveMode = active;
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public double getDefaultValue() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefaultValue(double defaultValue) {
    this.defaultValue = defaultValue;
    if (isActiveMode) {
      SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public double get() {
    return isActiveMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }
    return false;
  }
}
