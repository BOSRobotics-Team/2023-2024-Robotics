package frc.lib.util;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Gets a value from dashboard in active mode, returns default if not or value in dashboard.
 */
public class DashboardNumber {
  private String key = "";
  private double defaultValue = 0.0;
  private double lastHasChangedValue = defaultValue;

  /**
   * Create a new DashboardNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public DashboardNumber(String dashboardKey) {
    this.key = dashboardKey;
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
    if (!Preferences.containsKey(key)) {
      Preferences.setDouble(key, defaultValue);
    }
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public double get() {
    return Preferences.getDouble(key, defaultValue);
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public void set(double value) {
    Preferences.setDouble(key, value);
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
