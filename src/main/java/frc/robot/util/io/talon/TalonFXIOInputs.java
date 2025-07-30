package frc.robot.util.io.talon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIOInputs {
  public boolean connected = false;
  public Angle position = Rotations.of(0);
  public AngularVelocity velocity = RotationsPerSecond.of(0);
  public AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0);
  public Voltage appliedVoltage = Volts.of(0);
  public Current supplyCurrent = Amps.of(0), torqueCurrent = Amps.of(0);
  public Temperature temp = Celsius.of(0);
}
