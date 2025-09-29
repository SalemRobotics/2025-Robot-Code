package frc.robot.subsystems.algae_arm;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.algae_arm.AlgaeArmConstants.*;
import static frc.robot.util.phoenix.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public sealed class AlgaeArmIOTalonFX implements AlgaeArmIO permits AlgaeArmIOSim {
  protected final TalonFX talon = new TalonFX(MOTOR_ID, MOTOR_BUS);
  private final PositionTorqueCurrentFOC positionRequest =
      new PositionTorqueCurrentFOC(0).withSlot(0).withUpdateFreqHz(100).withFeedForward(Amps.of(4));

  private final StatusSignal<Angle> position = talon.getPosition();
  private final StatusSignal<AngularVelocity> velocity = talon.getVelocity();
  private Angle target = STOWED_ANGLE;

  public AlgaeArmIOTalonFX() {
    tryUntilOk(5, () -> talon.getConfigurator().apply(MOTOR_CONFIG));

    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AlgaeArmIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(position, velocity).isOK();

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.setpoint = target;
  }

  private void executeTarget(Angle target) {
    this.target = target;
    talon.setControl(positionRequest.withPosition(target));
  }

  @Override
  public void deploy() {
    executeTarget(DEPLOYED_ANGLE);
  }

  @Override
  public void stow() {
    executeTarget(STOWED_ANGLE);
  }
}
