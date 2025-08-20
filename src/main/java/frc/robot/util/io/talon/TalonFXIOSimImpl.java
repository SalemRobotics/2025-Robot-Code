package frc.robot.util.io.talon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class TalonFXIOSimImpl extends TalonFXIOImpl {
  private final DCMotorSim physicsSim;
  private double lastTimestamp = Timer.getTimestamp();
  private final TalonFXSimState simState;

  public TalonFXIOSimImpl(int port, CANBus bus, boolean enableFOC, double moi, double reduction) {
    super(port, bus);
    simState = talon.getSimState();

    if (!Utils.isSimulation()) {
      DriverStation.reportError("Attempting to create TalonFX sim on non-sim context", true);
    }

    DCMotor motor =
        (enableFOC ? DCMotor.getKrakenX60(1) : DCMotor.getKrakenX60Foc(1)).withReduction(reduction);

    physicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
  }

  public TalonFXIOSimImpl(
      int port,
      CANBus bus,
      boolean enableFOC,
      TalonFXConfiguration config,
      double moi,
      double reduction) {
    super(port, bus, config);
    simState = talon.getSimState();

    if (!Utils.isSimulation()) {
      DriverStation.reportError("Attempting to create TalonFX sim on non-sim context", true);
    }

    DCMotor motor =
        (enableFOC ? DCMotor.getKrakenX60(1) : DCMotor.getKrakenX60Foc(1)).withReduction(reduction);

    physicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
  }

  @Override
  public void updateInputs(TalonFXIOInputs inputs) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double appliedVoltage = simState.getMotorVoltage();
    physicsSim.setInputVoltage(appliedVoltage);

    var newTimestamp = Timer.getTimestamp();
    physicsSim.update(newTimestamp - lastTimestamp);
    lastTimestamp = newTimestamp;

    simState.setRawRotorPosition(physicsSim.getAngularPosition());
    simState.setRotorVelocity(physicsSim.getAngularVelocity());
    simState.setRotorAcceleration(physicsSim.getAngularAcceleration());

    // use the updates StatusSignals after running the physics sim
    super.updateInputs(inputs);
  }
}
