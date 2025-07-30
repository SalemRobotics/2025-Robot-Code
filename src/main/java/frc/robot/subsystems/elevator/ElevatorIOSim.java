package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public final class ElevatorIOSim extends ElevatorIOTalonFX {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(2).withReduction(9);
  // TODO: TUNE/CORRECT THESE CONSTANTS
  private final ElevatorSim physicsSim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              motor, 11.34, Units.inchesToMeters(6) / (2 * Math.PI), 1),
          motor,
          0,
          Units.inchesToMeters(6) * 4.67,
          true,
          0);

  private final TalonFXSimState simState = leader.getSimState();
  private double lastTimestamp = Timer.getTimestamp();

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double appliedVoltage = simState.getMotorVoltage();
    physicsSim.setInputVoltage(appliedVoltage);

    var newTimestamp = Timer.getTimestamp();
    physicsSim.update(newTimestamp - lastTimestamp);
    lastTimestamp = newTimestamp;

    var rots = ElevatorUtils.inchesToRotations(Meters.of(physicsSim.getPositionMeters()));
    simState.setRawRotorPosition(rots);
    var velocity =
        ElevatorUtils.convertVelocity(MetersPerSecond.of(physicsSim.getVelocityMetersPerSecond()));
    simState.setRotorVelocity(velocity);

    // use the updated StatusSignals after running the physics sim
    super.updateInputs(inputs);
  }
}
