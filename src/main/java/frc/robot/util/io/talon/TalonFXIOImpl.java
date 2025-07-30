package frc.robot.util.io.talon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Consumer;

public class TalonFXIOImpl implements TalonFXIO {
  protected final TalonFX talon;
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final CANBus canBus;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent, statorCurrent, torqueCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Integer> pidSlot;

  private final BaseStatusSignal[] signals;

  public TalonFXIOImpl(int port, CANBus bus) {
    talon = new TalonFX(port, bus);
    canBus = bus;

    position = talon.getPosition();
    velocity = talon.getVelocity();
    acceleration = talon.getAcceleration();
    voltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    statorCurrent = talon.getStatorCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temperature = talon.getDeviceTemp();
    pidSlot = talon.getClosedLoopSlot();

    signals =
        new BaseStatusSignal[] {
          position,
          velocity,
          acceleration,
          voltage,
          supplyCurrent,
          statorCurrent,
          torqueCurrent,
          temperature,
          pidSlot
        };
  }

  public TalonFXIOImpl(int port, CANBus bus, Consumer<TalonFXConfigurator> config) {
    this(port, bus);

    config.accept(talon.getConfigurator());
  }

  @Override
  public void stopMotor() {
    talon.stopMotor();
  }

  @Override
  public void setDutyCycle(double output) {
    talon.setControl(dutyCycleOut.withOutput(output));
  }

  @Override
  public void setPosition(Angle position) {
    talon.setControl(positionControl.withPosition(position));
  }

  @Override
  public void updateInputs(TalonFXIOInputs inputs) {
    StatusCode result;
    if (canBus.isNetworkFD()) {
      result = BaseStatusSignal.waitForAll(0.005, signals);
    } else {
      result = BaseStatusSignal.refreshAll(signals);
    }

    inputs.connected = result.isOK();
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.acceleration = acceleration.getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
  }
}
