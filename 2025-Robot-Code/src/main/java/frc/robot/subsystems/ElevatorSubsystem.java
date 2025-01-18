package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public static class ElevatorHeights {
        public static double Stowed = 0;
        public static double Coral_L1 = 1.0;
        public static double Coral_L2 = 2.0;
        public static double Coral_L3 = 3.0;
        public static double Coral_L4 = 4.0;
    }

    private final TalonFXConfiguration kConfig = new TalonFXConfiguration();
    private final TalonFX kElevatorMotorA = new TalonFX(ElevatorConstants.kElevatorMotorAPort,
            ElevatorConstants.kElevatorMotorBus);
    private final TalonFX kElevatorMotorB = new TalonFX(ElevatorConstants.kElevatorMotorBPort,
            ElevatorConstants.kElevatorMotorBus);
    private final MotionMagicVoltage kVoltage = new MotionMagicVoltage(0);

    public ElevatorSubsystem() {
        FeedbackConfigs fbcfg = kConfig.Feedback;
        fbcfg.SensorToMechanismRatio = ElevatorConstants.kSensorToMechanismRatio;

        MotionMagicConfigs mmcfg = kConfig.MotionMagic;
        mmcfg.withMotionMagicCruiseVelocity(RotationsPerSecond.of(ElevatorConstants.kElevatorMaxSpeed))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ElevatorConstants.kElevatorMaxAcceleration))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(ElevatorConstants.kElevatorMaxJerk));

        Slot0Configs slot0 = kConfig.Slot0;
        slot0.kS = ElevatorConstants.kElevatorS;
        slot0.kV = ElevatorConstants.kElevatorV;
        slot0.kA = ElevatorConstants.kElevatorA;
        slot0.kP = ElevatorConstants.kElevatorP;
        slot0.kI = ElevatorConstants.kElevatorI;
        slot0.kD = ElevatorConstants.kElevatorD;
        slot0.kG = ElevatorConstants.kElevatorG;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; i++) {
            status = kElevatorMotorA.getConfigurator().apply(kConfig);

            if (status.isOK())
                break;
        }

        if (!status.isOK())
            DataLogManager.log("Failed to configure elevator motors: " + status.toString());

        kElevatorMotorB.setControl(new Follower(kElevatorMotorA.getDeviceID(), true));
    }

    public Command setElevatorTarget(double height) {
        DataLogManager.log("Setting elevator height to " + height);
        return Commands.run(() -> kElevatorMotorA.setControl(kVoltage.withPosition(height).withSlot(0)));
    }
}
