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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mElevatorMotorA = new TalonFX(ElevatorConstants.kElevatorMotorAPort,
            ElevatorConstants.kElevatorMotorBus);
    private final TalonFX mElevatorMotorB = new TalonFX(ElevatorConstants.kElevatorMotorBPort,
            ElevatorConstants.kElevatorMotorBus);
    private final MotionMagicVoltage mVoltage = new MotionMagicVoltage(0);
    private double mSetHeight = 0;

    public Elevator() {
        FeedbackConfigs fbcfg = mConfig.Feedback;
        fbcfg.SensorToMechanismRatio = ElevatorConstants.kSensorToMechanismRatio;

        MotionMagicConfigs mmcfg = mConfig.MotionMagic;
        mmcfg.withMotionMagicCruiseVelocity(RotationsPerSecond.of(ElevatorConstants.kElevatorMaxSpeed))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ElevatorConstants.kElevatorMaxAcceleration))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(ElevatorConstants.kElevatorMaxJerk));

        Slot0Configs slot0 = mConfig.Slot0;
        slot0.kS = ElevatorConstants.kElevatorS;
        slot0.kV = ElevatorConstants.kElevatorV;
        slot0.kA = ElevatorConstants.kElevatorA;
        slot0.kP = ElevatorConstants.kElevatorP;
        slot0.kI = ElevatorConstants.kElevatorI;
        slot0.kD = ElevatorConstants.kElevatorD;
        slot0.kG = ElevatorConstants.kElevatorG;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; i++) {
            status = mElevatorMotorA.getConfigurator().apply(mConfig);

            if (status.isOK())
                break;
        }

        if (status.isError())
            DataLogManager.log("Failed to configure elevator motors: " + status.toString());

        mElevatorMotorB.setControl(new Follower(mElevatorMotorA.getDeviceID(), true));
    }

    public Command setElevatorTarget(double height) {
        mSetHeight = height;
        DataLogManager.log("Setting elevator height to " + height);
        return Commands.run(() -> mElevatorMotorA.setControl(mVoltage.withPosition(height).withSlot(0)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", mSetHeight);
    }
}
