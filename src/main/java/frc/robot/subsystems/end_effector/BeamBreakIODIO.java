package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIODIO implements BeamBreakIO {
  private final DigitalInput input;

  public BeamBreakIODIO(int channel) {
    input = new DigitalInput(channel);
  }

  @Override
  public boolean isBroken() {
    // for some reason, our sensors return false when they're broken??
    return !input.get();
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.isBroken = isBroken();
  }
}
