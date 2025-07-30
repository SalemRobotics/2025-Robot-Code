package frc.robot.util.io.talon;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TalonFXIO {
  public default void setDutyCycle(double output) {}

  public default void setPosition(Angle position) {}

  public default void stopMotor() {}

  public default void updateInputs(TalonFXIOInputs inputs) {}

  @AutoLog
  public static class TalonFXIOInputs {
    public boolean connected = false;
    public Angle position = Rotations.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0);
    public Voltage appliedVoltage = Volts.of(0);
    public Current supplyCurrent = Amps.of(0),
        statorCurrent = Amps.of(0),
        torqueCurrent = Amps.of(0);
    public Temperature temperature = Celsius.of(0);
  }
}
