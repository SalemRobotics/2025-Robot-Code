// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.PositionUtils.getDistance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double DRIVE_KP = 2.5;
  private static final double DRIVE_KD = 0.25;
  private static final double ANGLE_KP = 2.0;
  private static final double ANGLE_KD = 0.2;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Units.degreesToRadians(5);
  private static final double LINE_TOLERANCE = 0.05;
  private static final double POSITION_TOLERANCE = Units.inchesToMeters(1);
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static class JoystickApproachCommand extends Command {
    Drive drive;
    DoubleSupplier ySupplier;
    Supplier<Pose2d> targetSupplier;

    Pose2d targetPose2d;
    Pose2d currentPose2d;
    Pose2d relativePose2d;
    Rotation2d targetRotation2d;

    boolean running = false;

    static final double DEADBAND = 0.1;

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

    ProfiledPIDController alignController =
        new ProfiledPIDController(ANGLE_KP, 0, ANGLE_KD, new TrapezoidProfile.Constraints(3.7, 4));

    public JoystickApproachCommand(
        Drive drive, DoubleSupplier ySupplier, Supplier<Pose2d> targetSupplier) {
      this.drive = drive;
      this.ySupplier = ySupplier;
      this.targetSupplier = targetSupplier;

      angleController.setTolerance(POSITION_TOLERANCE);
      angleController.setTolerance(ANGLE_TOLERANCE);

      angleController.enableContinuousInput(-Math.PI, Math.PI);
      alignController.setGoal(0);

      addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      alignController.reset(0);
      angleController.reset(drive.getPose().getRotation().getRadians());
      targetPose2d = targetSupplier.get();

      Logger.recordOutput("AutoAlign/Approach/Target", targetPose2d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      running = true;
      relativePose2d = drive.getPose().relativeTo(targetPose2d);
      targetRotation2d = targetPose2d.getRotation();

      // Calculate lateral linear velocity
      Translation2d offsetVector =
          new Translation2d(0, alignController.calculate(relativePose2d.getY()));

      // Calculate total linear velocity
      Translation2d linearVelocity =
          getLinearVelocityFromJoysticks(-ySupplier.getAsDouble(), 0)
              .times(drive.getMaxLinearSpeedMetersPerSec())
              .plus(offsetVector)
              .rotateBy(targetRotation2d);

      // Calculate angular speed
      double omega =
          angleController.calculate(
              drive.getRotation().getRadians(),
              targetRotation2d.rotateBy(Rotation2d.k180deg).getRadians());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      running = false;
    }

    // Returns true when withing a lateral tolerance
    public boolean withinTolerance(double dist) {
      return running ? Math.abs(relativePose2d.getY()) < dist : false;
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
      // Apply deadband
      double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
      Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

      // Square magnitude for more precise control
      linearMagnitude = linearMagnitude * linearMagnitude;

      // Return new linear velocity
      return new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();
    }
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  @FunctionalInterface
  interface RotationFunc {
    public double getRotation(ProfiledPIDController controller, Pose2d current, Pose2d target);
  }

  static final RotationFunc standardApproachRotation =
      (controller, current, target) -> {
        final Rotation2d currentRot = current.getRotation(), targetRot = target.getRotation();

        double pidResult = controller.calculate(currentRot.getRadians(), targetRot.getRadians());
        System.out.println("StraightTowards::rotationFunc: pidResult = " + pidResult);

        double toGoal = targetRot.minus(currentRot).getRadians();
        System.out.println("StraightTowards::rotationFunc: toGoal = " + toGoal);

        double omega = toGoal * pidResult;
        System.out.println("StraightTowards::rotationFunc: omega = " + omega);

        return omega;
      };

  public static Command straightTowards(Drive drive, Supplier<Pose2d> target) {
    return straightTowards(drive, target, standardApproachRotation);
  }

  public static Command straightTowards(
      Drive drive, Supplier<Pose2d> targetSupplier, RotationFunc rotationFunc) {
    final ProfiledPIDController alignController =
        new ProfiledPIDController(
            DRIVE_KP,
            0,
            DRIVE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    alignController.setTolerance(LINE_TOLERANCE);
    alignController.setGoal(0);

    final ProfiledPIDController angleController =
        new ProfiledPIDController(
            DRIVE_KP,
            0,
            DRIVE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.setTolerance(ANGLE_TOLERANCE);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              final Pose2d robotPose = drive.getPose(), targetPose = targetSupplier.get();

              var robotToGoal = targetPose.minus(robotPose).getTranslation();

              double pidX = alignController.calculate(robotToGoal.getX());
              double pidY = alignController.calculate(robotToGoal.getY());

              System.out.println("StraightTowards::cmd: pidScalars = (" + pidX + ", " + pidY + ")");

              var pidTranslation =
                  new Translation2d(pidX * robotToGoal.getX(), pidY * robotToGoal.getY());

              var resultVector = robotToGoal.plus(pidTranslation);

              System.out.println(
                  "StraightTowards::cmd: final vector = ("
                      + resultVector.getX()
                      + ", "
                      + resultVector.getY()
                      + ")");

              final ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      resultVector.getX(),
                      resultVector.getY(),
                      rotationFunc.getRotation(angleController, robotPose, targetPose));

              drive.runVelocity(speeds);
            },
            drive)
        .until(() -> getDistance(drive, targetSupplier.get()) < POSITION_TOLERANCE);
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
