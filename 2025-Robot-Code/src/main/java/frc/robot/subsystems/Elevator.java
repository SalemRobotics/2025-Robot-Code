package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {

    private final TalonFXConfiguration mConfig = new TalonFXConfiguration();
    private final TalonFX mElevatorMotorA = new TalonFX(ElevatorConstants.kMotorAPort,
            ElevatorConstants.kMotorBus);
    private final TalonFX mElevatorMotorB = new TalonFX(ElevatorConstants.kMotorBPort,
            ElevatorConstants.kMotorBus);
    private final MotionMagicExpoVoltage mVoltage = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    private double mSetHeight = 0;
    public final Trigger kIsStowed = new Trigger(() -> MathUtil.isNear(0,
            mElevatorMotorA.getPosition().getValueAsDouble(), ElevatorConstants.kPositionTolerance));

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", mElevatorMotorA.getPosition().getValueAsDouble());
    }

    public Elevator() {
        mConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kSensorToMechanismRatio;

        mConfig.CurrentLimits.withStatorCurrentLimit(ElevatorConstants.kStatorCurrentLimit)
                .withSupplyCurrentLimit(ElevatorConstants.kSupplyCurrentLimit);

        MotionMagicConfigs mmcfg = mConfig.MotionMagic;
        mmcfg
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(ElevatorConstants.kMaxSpeed))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ElevatorConstants.kMaxAcceleration))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(ElevatorConstants.kMaxJerk))
                .withMotionMagicExpo_kA(0.4)
                .withMotionMagicExpo_kV(ElevatorConstants.kV);

        Slot0Configs slot0 = mConfig.Slot0;
        slot0.withGravityType(GravityTypeValue.Elevator_Static);
        slot0.kS = ElevatorConstants.kS;
        slot0.kV = ElevatorConstants.kV;
        slot0.kA = ElevatorConstants.kA;
        slot0.kP = ElevatorConstants.kP;
        slot0.kI = ElevatorConstants.kI;
        slot0.kD = ElevatorConstants.kD;
        slot0.kG = ElevatorConstants.kG;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; i++) {
            status = mElevatorMotorA.getConfigurator().apply(mConfig);

            if (status.isOK())
                break;
        }

        if (status.isError())
            System.err.println("Failed to configure elevator motors: " + status.toString());

        mElevatorMotorB.setControl(new Follower(mElevatorMotorA.getDeviceID(), true));
        mElevatorMotorA.setPosition(0);
        mElevatorMotorB.setPosition(0);
    }

    public Command setElevatorTarget(double height) {
        return run(() -> {
            mSetHeight = height;
            mElevatorMotorA.setControl(mVoltage.withPosition(height).withSlot(0));
        });
    }

    public boolean isAtHeight() {
        return MathUtil.isNear(mSetHeight, mElevatorMotorA.getPosition().getValueAsDouble(),
                ElevatorConstants.kPositionTolerance);
    }
}
