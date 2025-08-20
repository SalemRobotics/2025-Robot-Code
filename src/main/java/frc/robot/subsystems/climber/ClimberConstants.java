package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
  public static final int kMotorId = 30;
  public static final int kServoId = 0;

  public static final TalonFXConfiguration kMotorConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true));
}
