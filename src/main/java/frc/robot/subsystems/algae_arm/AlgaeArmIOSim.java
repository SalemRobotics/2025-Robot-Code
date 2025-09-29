package frc.robot.subsystems.algae_arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.algae_arm.AlgaeArmConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import lombok.val;

public final class AlgaeArmIOSim extends AlgaeArmIOTalonFX {
  private final SingleJointedArmSim physicsSim;
  private final TalonFXSimState simState;
  private double lastTimestamp = Timer.getTimestamp();

  public AlgaeArmIOSim() {
    super();
    simState = talon.getSimState();

    DCMotor motor = DCMotor.getKrakenX60Foc(1).withReduction(SYSTEM_REDUCTION);
    // the reduction is already accounted for, so set it to 1
    val linearSystem = LinearSystemId.createSingleJointedArmSystem(motor, kMoi, 1);
    physicsSim =
        new SingleJointedArmSim(
            linearSystem,
            motor,
            1,
            kArmLength.in(Meters),
            STOWED_ANGLE.in(Radians),
            DEPLOYED_ANGLE.in(Radians),
            true,
            0);
  }

  @Override
  public void updateInputs(AlgaeArmIOInputs inputs) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simState.getMotorVoltage());

    double newTimestamp = Timer.getTimestamp();
    physicsSim.update(newTimestamp - lastTimestamp);
    lastTimestamp = newTimestamp;

    double rotorPosition = Units.radiansToRotations(physicsSim.getAngleRads());
    simState.setRawRotorPosition(rotorPosition);
    double rotorVelocity = Units.radiansToRotations(physicsSim.getVelocityRadPerSec());
    simState.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
