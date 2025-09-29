package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberConstants {
  public static final int MOTOR_ID = 30;
  public static final int SERVO_ID = 0;
  public static final CANBus MOTOR_BUS = Constants.kCanivore;

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true));
}
