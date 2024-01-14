package frc.lib.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;

public class GyroIOPigeon2Phoenix6 implements GyroIO {
  private final Pigeon2 gyro;
  private final StatusSignal<Double> yawStatusSignal;
  private final StatusSignal<Double> pitchStatusSignal;
  private final StatusSignal<Double> rollStatusSignal;
  private final StatusSignal<Double> angularVelocityXStatusSignal;
  private final StatusSignal<Double> angularVelocityYStatusSignal;
  private final StatusSignal<Double> angularVelocityZStatusSignal;
  private final Pigeon2SimState gyroSim;

  public GyroIOPigeon2Phoenix6(int id, String canBus) {
    this.gyro = new Pigeon2(id, canBus);

    /* Configure Pigeon2 */
    var toApply = new Pigeon2Configuration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    this.gyro.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    this.yawStatusSignal = this.gyro.getYaw().clone();
    this.yawStatusSignal.setUpdateFrequency(100);
    this.pitchStatusSignal = this.gyro.getPitch().clone();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.gyro.getRoll().clone();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityXStatusSignal = this.gyro.getAngularVelocityXWorld().clone();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.gyro.getAngularVelocityYWorld().clone();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);
    this.angularVelocityZStatusSignal = this.gyro.getAngularVelocityZWorld().clone();
    this.angularVelocityZStatusSignal.setUpdateFrequency(100);

    if (Robot.isSimulation()) {
      this.gyroSim = this.gyro.getSimState();
    } else {
      this.gyroSim = null;
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        this.yawStatusSignal,
        this.angularVelocityZStatusSignal,
        this.pitchStatusSignal,
        this.rollStatusSignal,
        this.angularVelocityXStatusSignal,
        this.angularVelocityYStatusSignal);

    inputs.connected = (this.yawStatusSignal.getStatus() == StatusCode.OK);
    inputs.positionDeg = gyro.getAngle(); //  inputs.yawDeg;
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.yawStatusSignal, this.angularVelocityZStatusSignal);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.pitchStatusSignal, this.angularVelocityYStatusSignal);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.rollStatusSignal, this.angularVelocityXStatusSignal);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue();
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue();
    inputs.yawDegPerSec = this.angularVelocityZStatusSignal.getValue();
    if (Robot.isSimulation()) {
      this.gyroSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
  }

  @Override
  public boolean isConnected() {
    if (Robot.isReal()) {
      return (this.yawStatusSignal.getStatus() == StatusCode.OK);
    }
    return false;
  }

  @Override
  public void setYaw(double yaw) {
    this.gyro.setYaw(yaw, 0.1);
  }

  @Override
  public void addYaw(double yaw) {
    if (Robot.isSimulation()) {
      this.gyroSim.addYaw(yaw);
    }
  }

  @Override
  public List<StatusSignal<Double>> getOdometryStatusSignals() {
    ArrayList<StatusSignal<Double>> signals = new ArrayList<>();
    signals.add(this.yawStatusSignal);
    signals.add(this.angularVelocityZStatusSignal);
    return signals;
  }
}
