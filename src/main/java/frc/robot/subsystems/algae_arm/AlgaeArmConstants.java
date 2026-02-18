package frc.robot.subsystems.algae_arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public final class AlgaeArmConstants {
  public static final int MOTOR_ID = 21;
  public static final CANBus MOTOR_BUS = Constants.kRIOBus;
  public static final int SYSTEM_REDUCTION = 25;

  public static final Angle STOWED_ANGLE = Rotations.of(0);
  public static final Angle DEPLOYED_ANGLE = Rotations.of(0.283);
  // TODO: CORRECT THIS VALUE WITH CAD
  public static final Distance kArmLength = Inches.of(9);
  // TODO: CORRECT THIS VALUE (ASK BRENNAN/MARK)
  public static final double kMoi = 0.4;

  public static final CurrentLimitsConfigs CURRENT_LIMITS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(10))
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(40))
          .withSupplyCurrentLimitEnable(true);
  public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs().withSensorToMechanismRatio(SYSTEM_REDUCTION);
  public static final MotorOutputConfigs MOTOR_OUTPUT =
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
  public static final Slot0Configs SLOT_0 =
      new Slot0Configs()
          .withKP(10)
          .withKI(0)
          .withKD(1)
          .withKS(0)
          .withKV(1)
          .withKA(0.05)
          .withKG(0.5)
          .withGravityType(GravityTypeValue.Arm_Cosine);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(CURRENT_LIMITS)
          .withFeedback(FEEDBACK_CONFIGS)
          .withMotorOutput(MOTOR_OUTPUT)
          .withSlot0(SLOT_0);
}
