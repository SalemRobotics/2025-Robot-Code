package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.phoenix.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.Elevator.Setpoint;

public sealed class ElevatorIOTalonFX implements ElevatorIO permits ElevatorIOSim {
  protected final TalonFX leader = new TalonFX(kLeaderId, kMotorBus);
  private final TalonFX follower = new TalonFX(kFollowerId, kMotorBus);

  private final StatusSignal<Angle> leaderPosition = leader.getPosition(),
      followerPosition = follower.getPosition();
  private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
  private final StatusSignal<AngularAcceleration> acceleration = leader.getAcceleration();
  private final StatusSignal<Voltage> leaderVoltage = leader.getMotorVoltage(),
      followerVoltage = follower.getMotorVoltage();
  private final StatusSignal<Current> torqueCurrent = leader.getTorqueCurrent(),
      statorCurrent = leader.getStatorCurrent(),
      supplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Double> closedLoopError = leader.getClosedLoopError(),
      closedLoopReference = leader.getClosedLoopReference(),
      closedLoopRefSlope = leader.getClosedLoopReferenceSlope(),
      closedLoopProportional = leader.getClosedLoopProportionalOutput(),
      closedLoopDerivative = leader.getClosedLoopDerivativeOutput(),
      closedLoopFeedForward = leader.getClosedLoopFeedForward(),
      closedLoopOutput = leader.getClosedLoopOutput();

  private final BaseStatusSignal[] leaderSignals =
      new BaseStatusSignal[] {
        leaderPosition,
        velocity,
        acceleration,
        leaderVoltage,
        torqueCurrent,
        statorCurrent,
        supplyCurrent,
        closedLoopError,
        closedLoopReference,
        closedLoopRefSlope,
        closedLoopProportional,
        closedLoopDerivative,
        closedLoopFeedForward,
        closedLoopOutput
      };

  /** Control request for moving up to a setpoint, or for stowing the carriage */
  private final MotionMagicExpoTorqueCurrentFOC exponential =
      new MotionMagicExpoTorqueCurrentFOC(0).withUseTimesync(true).withFeedForward(Amps.of(8));
  /** Control request for moving between setpoints, where neither is from stow */
  private final DynamicMotionMagicTorqueCurrentFOC trapezoidal =
      new DynamicMotionMagicTorqueCurrentFOC(
              Rotations.of(0),
              Trapezoidal.kMaxVelocity,
              Trapezoidal.kMaxAcceleration,
              Trapezoidal.kMaxJerk)
          .withUseTimesync(true)
          .withFeedForward(Amps.of(8));

  public ElevatorIOTalonFX() {
    final var config =
        new TalonFXConfiguration()
            .withSlot0(kRiseGains)
            .withSlot1(kStowGains)
            .withSlot2(kTraversalGains)
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(120)
                    .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0)
                    .withMotionMagicAcceleration(Exponential.kMaxAcceleration)
                    .withMotionMagicJerk(Exponential.kMaxJerk)
                    .withMotionMagicExpo_kV(Exponential.kExpo_kV)
                    .withMotionMagicExpo_kA(Exponential.kExpo_kA))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(9))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withReverseSoftLimitThreshold(0)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Setpoint.Barge.target)
                    .withForwardSoftLimitEnable(true));

    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    tryUntilOk(5, () -> follower.getConfigurator().apply(config));
    tryUntilOk(5, () -> follower.setControl(new Follower(kLeaderId, true)));

    leader.setPosition(0);
    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setPosition(0);
    follower.setNeutralMode(NeutralModeValue.Brake);

    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(250), followerPosition, followerVoltage);
    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(250), leaderSignals);

    BaseStatusSignal.waitForAll(1, leaderSignals);
    BaseStatusSignal.waitForAll(1, followerPosition, followerVoltage);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderConnected = BaseStatusSignal.waitForAll(0.005, leaderSignals).isOK();
    inputs.followerConnected =
        BaseStatusSignal.waitForAll(0.005, followerPosition, followerVoltage).isOK();

    inputs.leaderPosition = leaderPosition.getValue();
    inputs.followerPosition = followerPosition.getValue();
    inputs.height = ElevatorUtils.rotsToInches(inputs.leaderPosition);
    inputs.velocity = velocity.getValue();
    inputs.acceleration = acceleration.getValue();
    inputs.leaderVoltage = leaderVoltage.getValue();
    inputs.followerVoltage = followerVoltage.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();

    inputs.closedLoopError = closedLoopError.getValue();
    inputs.closedLoopReference = closedLoopReference.getValue();
    inputs.closedLoopRefSlope = closedLoopRefSlope.getValue();
    inputs.proportionalOutput = closedLoopProportional.getValue();
    inputs.derivativeOutput = closedLoopDerivative.getValue();
    inputs.feedforward = closedLoopFeedForward.getValue();
    inputs.closedLoopOutput = closedLoopOutput.getValue();
  }

  @Override
  public void setTarget(Angle angle, boolean useTrapezoidal) {
    ControlRequest req;

    if (useTrapezoidal) {
      req = trapezoidal.withPosition(angle).withSlot(2);
    } else {
      req = exponential.withPosition(angle).withSlot(0);
    }

    leader.setControl(req);
  }

  @Override
  public void stow() {
    leader.setControl(exponential.withPosition(0).withSlot(1));
  }
}
