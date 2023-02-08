package frc.lib.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Gets a value from dashboard in active mode, returns default if not or value in dashboard.
 */
public class DashboardNumber {
  public static boolean isActiveMode = false;

  private static final String TABLE_KEY = "Preferences";

  private ShuffleboardTab tab = Shuffleboard.getTab(TABLE_KEY);
  private SimpleWidget widget = null;
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
      this.widget = tab.add(key, defaultValue).withSize(2, 1);
    }
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public double get() {
    if (isActiveMode) {
      return this.widget != null ? this.widget.getEntry().getDouble(defaultValue) : defaultValue;
    }
    return defaultValue;
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public void set(double value) {
    if (isActiveMode) {
      this.widget.getEntry().setDouble(value);
    }
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
