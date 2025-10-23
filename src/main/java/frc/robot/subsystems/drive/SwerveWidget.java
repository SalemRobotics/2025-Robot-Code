package frc.robot.subsystems.drive;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public final class SwerveWidget implements Sendable {
  private final Drive drive;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    // Initialize the front left module
    builder.addDoubleProperty(
        "Front Left Angle", () -> drive.getModulePositions()[0].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Velocity", () -> drive.getModuleStates()[0].speedMetersPerSecond, null);

    // Initialize the front right module
    builder.addDoubleProperty(
        "Front Right Angle", () -> drive.getModulePositions()[1].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Velocity", () -> drive.getModuleStates()[1].speedMetersPerSecond, null);

    // Initialize the back left module
    builder.addDoubleProperty(
        "Back Left Angle", () -> drive.getModulePositions()[2].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Left Velocity", () -> drive.getModuleStates()[2].speedMetersPerSecond, null);

    // Initialize the back right module
    builder.addDoubleProperty(
        "Back Right Angle", () -> drive.getModulePositions()[3].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Right Velocity", () -> drive.getModuleStates()[3].speedMetersPerSecond, null);
  }
}
