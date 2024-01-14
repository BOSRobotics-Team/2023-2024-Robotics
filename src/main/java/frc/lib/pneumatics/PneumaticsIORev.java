package frc.lib.pneumatics;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticHub;

/**
 * Implementation of PneumaticsIO for REV Robotics Pneumatics Hub, dual REV Robotics pressure
 * sensors, and an analog flow sensor (i.e., SMC PFM711-N7-C-R.)
 */
public class PneumaticsIORev implements PneumaticsIO {

  private final PneumaticHub pneumatics;

  // private final AnalogInput flowSensor;

  public PneumaticsIORev(int moduleID) {
    pneumatics = new PneumaticHub(moduleID);
    // flowSensor = new AnalogInput(FLOW_SENSOR_CHANNEL);
    useLowClosedLoopThresholds(false);
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    // inputs.highPressurePSI = pneumatics.getPressure(REV_HIGH_PRESSURE_SENSOR_CHANNEL);
    // inputs.lowPressurePSI = pneumatics.getPressure(REV_LOW_PRESSURE_SENSOR_CHANNEL);
    inputs.compressorActive = pneumatics.getCompressor();
    inputs.compressorCurrentAmps = pneumatics.getCompressorCurrent();

    /*
     * Our SMC flow sensor (PFM711-N7-C-R) provides analog output from 1V to 5V.
     * 1V corresponds to 0 L/min; 5V corresponds to 100 L/min.
     */
    // inputs.flowLPM = ((flowSensor.getAverageVoltage() * 25) - 25);
    // inputs.volumeL += (inputs.flowLPM * LOOP_PERIOD_SECS) / 60;
  }

  @Override
  public void useLowClosedLoopThresholds(boolean useLow) {
    // if (useLow) {
    //   pneumatics.enableCompressorAnalog(MIN_LOW_PRESSURE, MAX_LOW_PRESSURE);
    // } else {
    //   pneumatics.enableCompressorAnalog(MIN_HIGH_PRESSURE, MAX_HIGH_PRESSURE);
    // }
  }
}
