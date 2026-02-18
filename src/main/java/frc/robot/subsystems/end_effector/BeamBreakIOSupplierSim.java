package frc.robot.subsystems.end_effector;

import java.util.function.BooleanSupplier;

public class BeamBreakIOSupplierSim implements BeamBreakIO {
  private final BooleanSupplier supplier;

  public BeamBreakIOSupplierSim(BooleanSupplier supplier) {
    this.supplier = supplier;
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.isBroken = supplier.getAsBoolean();
  }
}
