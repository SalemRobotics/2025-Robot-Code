package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LoggedTunablePID extends ProfiledPIDController {
  @SuppressWarnings("unused")
  private final String name;

  private final LoggedTunableNumber kP, kI, kD, iZone, maxV, maxA;

  public LoggedTunablePID(String name, double p, double i, double d, double maxV, double maxA) {
    this(name, p, i, d, maxV, maxA, Double.POSITIVE_INFINITY);
  }

  public LoggedTunablePID(
      String name, double p, double i, double d, double iZone, double maxV, double maxA) {
    super(p, i, d, new TrapezoidProfile.Constraints(maxV, maxA));
    setIZone(iZone);

    this.name = name;

    kP = new LoggedTunableNumber(name + "/P", p);
    kI = new LoggedTunableNumber(name + "/I", i);
    kD = new LoggedTunableNumber(name + "/D", d);
    this.iZone = new LoggedTunableNumber(name + "/iZone", iZone);

    this.maxV = new LoggedTunableNumber(name + "/MaxVelocity", maxV);
    this.maxA = new LoggedTunableNumber(name + "/MaxAcceleration", maxA);
  }

  public void updatePID() {
    // If changed, update controller constants from Tuneable Numbers
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      setPID(kP.get(), kI.get(), kD.get());
    }

    if (iZone.hasChanged(hashCode())) {
      setIZone(iZone.get());
    }

    if (maxV.hasChanged(hashCode()) || maxA.hasChanged(hashCode())) {
      setConstraints(new TrapezoidProfile.Constraints(maxV.get(), maxA.get()));
    }
  }
}
