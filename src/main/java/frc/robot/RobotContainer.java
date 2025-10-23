// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.CAMERA_NAMES;
import static frc.robot.subsystems.vision.VisionConstants.ROBOT_TO_CAMERAS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.autopilot.AutopilotCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae_arm.AlgaeArm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Setpoint;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.end_effector.BeamBreakIO;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.PositionUtils;
import frc.robot.util.io.talon.TalonFXIO;
import java.util.Set;
import java.util.function.Supplier;
import lombok.val;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final EndEffector endEffector;
  private final Elevator elevator;
  private final AlgaeArm algaeArm;
  private final Climber climber;

  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  private static enum ControlMode {
    Coral,
    Algae,
  }

  private ControlMode controlMode = ControlMode.Coral;

  private final Trigger coralMode = new Trigger(() -> controlMode == ControlMode.Coral);
  private final Trigger algaeMode = new Trigger(() -> controlMode == ControlMode.Algae);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(CAMERA_NAMES[0], ROBOT_TO_CAMERAS[0]),
                new VisionIOPhotonVision(CAMERA_NAMES[1], ROBOT_TO_CAMERAS[1]));
        endEffector = EndEffector.createReal();
        elevator = new Elevator(new ElevatorIOTalonFX());
        algaeArm = AlgaeArm.createReal();
        climber = new Climber(new ClimberIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(CAMERA_NAMES[0], ROBOT_TO_CAMERAS[0], drive::getPose),
                new VisionIOPhotonVisionSim(CAMERA_NAMES[1], ROBOT_TO_CAMERAS[1], drive::getPose));

        endEffector = EndEffector.createSim();
        elevator = new Elevator(new ElevatorIOSim());
        algaeArm = AlgaeArm.createSim();
        // We don't simulate the climber due to the complexity of how it works and the ramifications
        // it
        // has on the rest of the robot
        climber = new Climber(inputs -> {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive, inputs -> {}, inputs -> {});
        endEffector =
            new EndEffector(new TalonFXIO() {}, new BeamBreakIO() {}, new BeamBreakIO() {}, false);
        elevator = new Elevator(new ElevatorIO() {});
        algaeArm = new AlgaeArm(new TalonFXIO() {});
        climber = new Climber(inputs -> {});
        break;
    }

    superstructure = new Superstructure(endEffector, elevator, algaeArm);

    // configure auto commands BEFORE auto initializations :P
    configureAutoCommands();
    PathfindingCommand.warmupCommand().schedule();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Only need to initialize mirrored paths because own side is included by buildAutoChooser
    autoChooser.addOption("Opps Side 4pc", new PathPlannerAuto("Own Side 4pc", true));

    // Only initialize dev autos while working with a devbot
    if (Constants.DEVBOT) {
      // Set up SysId routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData(
        "Pathfind To Auto", DriveCommands.driveToAutoStart(drive, autoChooser::get));
  }

  public void periodic() {
    field.setRobotPose(drive.getPose());

    Logger.recordOutput("Robot/Control Mode", controlMode);

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void disabledPeriodic() {
    val auto = autoChooser.get();

    if (auto instanceof PathPlannerAuto a) {
      Logger.recordOutput(
          "Robot/Near Auto Start",
          PositionUtils.isNear(drive.getPose(), a.getStartingPose(), 0.04));
    } else {
      Logger.recordOutput("Robot/Near Auto Start", false);
    }
  }

  /**
   * Use this method to define your button to command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Initialize coral scoring
    controller.rightTrigger().and(coralMode).whileTrue(superstructure.scoreCoral(false));

    // deferred stow command (so the setpoint can be changed)
    var deferredStow =
        Commands.defer(
                () -> Commands.waitSeconds(SmartDashboard.getNumber("Elevator Defer Timeout", 0.1)),
                Set.of())
            .andThen(elevator.setTarget(Setpoint.Stowed));

    // Initialize elevator setpoints
    controller.x().and(coralMode).whileTrue(elevator.setTarget(Setpoint.L2)).onFalse(deferredStow);
    controller
        .x()
        .and(algaeMode)
        .whileTrue(elevator.setTarget(Setpoint.AlgaeLow))
        .onFalse(deferredStow);
    controller.b().and(coralMode).whileTrue(elevator.setTarget(Setpoint.L3)).onFalse(deferredStow);
    controller
        .b()
        .and(algaeMode)
        .whileTrue(elevator.setTarget(Setpoint.AlgaeHigh))
        .onFalse(deferredStow);
    controller.y().and(coralMode).whileTrue(elevator.setTarget(Setpoint.L4)).onFalse(deferredStow);
    controller.y().and(algaeMode).whileTrue(superstructure.bargeShot()).onFalse(deferredStow);

    // Initialize coral mode auto align
    controller
        .leftBumper()
        .and(coralMode)
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));
    controller
        .rightBumper()
        .and(coralMode)
        .whileTrue(
            joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));

    // Initialize algae mode auto align
    controller
        .leftBumper()
        .and(algaeMode)
        .whileTrue(joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose())));
    controller
        .rightBumper()
        .and(algaeMode)
        .whileTrue(joystickApproach(() -> FieldConstants.getNearestReefFace(drive.getPose())));
    controller
        .a()
        .and(algaeMode)
        .whileTrue(AutopilotCommands.driveToBarge(drive, () -> -controller.getLeftX()));

    // Initialize barge algae mode & processor scoring
    controller
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> controlMode = ControlMode.Algae))
        .whileTrue(Commands.print("Algae mode!").andThen(superstructure.algaeMode()))
        .onFalse(
            Commands.runOnce(() -> controlMode = ControlMode.Coral).alongWith(algaeArm.stow()));

    controller.rightTrigger().and(algaeMode).whileTrue(endEffector.scoreProcessor());

    // Initialize climbing commands
    controller.povDown().whileTrue(climber.deploy());
    controller.povUp().whileTrue(climber.retract()).onTrue(algaeArm.drop());

    controller
        .povLeft()
        .whileTrue(AutopilotCommands.intakeCoral(true, drive, elevator, endEffector));
    controller
        .povRight()
        .whileTrue(AutopilotCommands.intakeCoral(false, drive, elevator, endEffector));
  }

  // Auto NamedCommands instantiations
  private void configureAutoCommands() {
    NamedCommands.registerCommand("elevator_stow", elevator.setTarget(Setpoint.Stowed));
    NamedCommands.registerCommand("elevator_l3", elevator.setTarget(Setpoint.L3));
    NamedCommands.registerCommand("elevator_l4", elevator.setTarget(Setpoint.L4));
    NamedCommands.registerCommand(
        "coral_intake", Commands.race(endEffector.autoIntake(), Commands.waitSeconds(0.5)));
    NamedCommands.registerCommand("coral_jog", endEffector.autoJogCoral());
    NamedCommands.registerCommand("score_coral", superstructure.scoreCoral(true));
    NamedCommands.registerCommand("score_barge", superstructure.bargeShot());
  }

  private Command joystickApproach(Supplier<Pose2d> approach) {
    return new DriveCommands.JoystickApproachCommand(drive, () -> -controller.getLeftY(), approach);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Start the command to stow the elevator. This is so that autonomous doesn't exit with the
   * elevator setpoint non-stowed
   */
  public void stowElevator() {
    elevator.setTarget(Setpoint.Stowed).schedule();
  }
}
