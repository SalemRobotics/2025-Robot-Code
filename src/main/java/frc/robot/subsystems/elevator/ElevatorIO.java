package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.elevator.Elevator.Setpoint;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leaderConnected = false, followerConnected = false;
    public Angle leaderPosition = Rotations.of(0), followerPosition = Rotations.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0);
    public Voltage leaderVoltage = Volts.of(0), followerVoltage = Volts.of(0);
    public Current torqueCurrent = Amps.of(0),
        statorCurrent = Amps.of(0),
        supplyCurrent = Amps.of(0);
    public double closedLoopError = 0, closedLoopReference = 0, closedLoopRefSlope = 0;
    public double proportionalOutput = 0,
        derivativeOutput = 0,
        feedforward = 0,
        closedLoopOutput = 0;
    public Distance height = Inches.of(0);
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTarget(Setpoint setpoint) {}
}
