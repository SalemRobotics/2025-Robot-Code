package frc.robot.subsystems.algae_arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.phoenix.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import java.util.function.Consumer;

public final class AlgaeArmConstants {
  public static final int kMotorID = 21;
  public static final CANBus kBus = Constants.kRIOBus;
  public static final int kReduction = 25;

  public static final Angle kStowedAngle = Rotations.of(0);
  public static final Angle kDeployedAngle = Rotations.of(0.283);
  // TODO: CORRECT THIS VALUE WITH CAD
  public static final Distance kArmLength = Inches.of(9);
  // TODO: CORRECT THIS VALUE (ASK BRENNAN/MARK)
  public static final double kMoi = 0.4;

  public static final CurrentLimitsConfigs kCurrentLimits =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(10))
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(40))
          .withSupplyCurrentLimitEnable(true);
  public static final FeedbackConfigs kFeedback =
      new FeedbackConfigs().withSensorToMechanismRatio(kReduction);
  public static final MotionMagicConfigs kMotionMagic = new MotionMagicConfigs();
  public static final Slot0Configs kSlot0 =
      new Slot0Configs()
          .withKP(10)
          .withKI(0)
          .withKD(1)
          .withKS(0)
          .withKV(1)
          .withKA(0.05)
          .withKG(0.5)
          .withGravityType(GravityTypeValue.Arm_Cosine);

  public static final Consumer<TalonFXConfigurator> kAlgaeArmCallback =
      config -> {
        tryUntilOk(5, () -> config.apply(kSlot0));
        tryUntilOk(5, () -> config.apply(kCurrentLimits));
        tryUntilOk(5, () -> config.apply(kFeedback));
      };
}
