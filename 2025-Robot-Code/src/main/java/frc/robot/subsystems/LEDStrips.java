package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDStrips extends SubsystemBase {
    /*
     * - On, without driver station: Slow flash of red (5 sec interval)
     * - On, with driver station, without FMS: Solid orange
     * - DS connected, in auto, diabled, & not aligned to auto starting point: Solid
     * color(TBD) only in the direction it needs to move in
     * - DS connected, Enabled in auto: Alliance color a bouncing white bar
     * - Tele-op: Alliance color
     * - (In tele-op) Gets a coral: Green pulse twice
     * - (In tele-op) Hits robot speed limit: Solid white
     * - Tele-op ends but before disconnection from DS: Turn LEDs off
     */
    private final AddressableLED mLEDStrip;
    private final AddressableLEDBuffer mLEDBuffer;
    private final AddressableLEDBufferView mLeftBufferView;
    private final AddressableLEDBufferView mRightBufferView;

    public LEDStrips() {
        mLEDStrip = new AddressableLED(LEDConstants.kLEDPort);
        mLEDStrip.setLength(LEDConstants.kLength);

        mLEDBuffer = new AddressableLEDBuffer(LEDConstants.kLength);
        mLeftBufferView = mLEDBuffer.createView(0, LEDConstants.kLEDSplitPosition - 1)
                .reversed();
        mRightBufferView = mLEDBuffer.createView(LEDConstants.kLEDSplitPosition, LEDConstants.kLength - 1);
    }

    @Override
    public void periodic() {
        mLEDStrip.setData(mLEDBuffer);
    }

    private Color getAllianceColor() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue ? Color.kBlue : Color.kRed;
    }

    public Command onMissingDriverStation() {
        return run(() -> {
            LEDPattern base = LEDPattern.solid(Color.kRed);
            LEDPattern pattern = base.blink(Seconds.of(0.5), Seconds.of(2));
            pattern.applyTo(mLEDBuffer);
        });
    }

    public Command onMissingFMS() {
        return run(() -> {
            LEDPattern color = LEDPattern.solid(Color.kOrange);
            color.applyTo(mLEDBuffer);
        });
    }

    /**
     * 
     * @param direction the direction the auto needs to go in: true = left, false =
     *                  right
     * @return
     */
    public Command onMisalignedAuto(boolean direction) {
        return run(() -> {
            var buffer = direction ? mLeftBufferView : mRightBufferView;
            LEDPattern color = LEDPattern.solid(Color.kViolet);
            color.applyTo(buffer);
        });
    }

    public Command onEnabledInAuto() {
        return run(() -> {
            Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite);
            Color allianceColor = getAllianceColor();

            LEDPattern base = LEDPattern.solid(allianceColor);
            LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));

            LEDPattern pattern = base.mask(mask);
            pattern.applyTo(mLEDBuffer);
        });
    }

    public Command onTeleop() {
        return run(() -> {
            Color allianceColor = getAllianceColor();
            LEDPattern color = LEDPattern.solid(allianceColor);
            color.applyTo(mLEDBuffer);
        });
    }

    public Command onCoralReceived() {
        return run(() -> {
            LEDPattern color = LEDPattern.solid(Color.kLimeGreen);
            color.applyTo(mLEDBuffer);
        });
    }

    public Command onSpeedLimitHit() {
        return run(() -> {
            LEDPattern color = LEDPattern.solid(Color.kWhite);
            color.applyTo(mLEDBuffer);
        });
    }

    public Command disableLights() {
        return run(() -> {
            LEDPattern color = LEDPattern.solid(Color.kBlack);
            color.applyTo(mLEDBuffer);
        });
    }
}
