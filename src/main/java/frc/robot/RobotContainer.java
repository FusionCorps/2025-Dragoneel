// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ScoringPieceType;
import frc.robot.Constants.TargetState;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorAndWristCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.AutoAlignDirection;
import frc.robot.subsystems.drive.DriveConstants.DriveSpeedMode;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.module.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSparkFlex;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.AutoLogOutput;
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
  private final Elevator elevator;
  private final Drive drive;
  private SwerveDriveSimulation driveSim; // only used in simulation for maple-sim swerve drive
  private final Vision vision;
  private final Climb climb;
  private final Shooter shooter;
  private final Wrist wrist;

  // elevator and wrist movmentcommands
  private ElevatorAndWristCommands elevatorAndWristCommands;

  // dashboard selector for autonomous period routines
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  // driver controller
  static final CommandXboxController controller =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  final Alert controllerDisconnectedAlert = new Alert("Controller Disconnected.", AlertType.kError);

  @AutoLogOutput public static TargetState targetPosition = TargetState.STATION;
  @AutoLogOutput public static ScoringPieceType currentScoringPieceType = ScoringPieceType.CORAL;

  // Trigger for current scoring piece type (coral/algae)
  private Trigger isCoralMode =
      new Trigger(() -> currentScoringPieceType == ScoringPieceType.CORAL);

  /** The container for the robot. Contains subsystems, operator devices, and commands. */
  public RobotContainer() {
    // Instantiate subsystems
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFXReal(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFXReal(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFXReal(DriveConstants.BACK_RIGHT));
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    CAM_FL_NAME, ROBOT_TO_CAM_FL_TRANSFORM, drive::getRotation),
                new VisionIOPhotonVision(
                    CAM_FR_NAME, ROBOT_TO_CAM_FR_TRANSFORM, drive::getRotation));
        climb = new Climb(new ClimbIOTalonFX());
        shooter = new Shooter(new ShooterIOSparkFlex());
        elevator = new Elevator(new ElevatorIOTalonFX());
        wrist = new Wrist(new WristIOSparkFlex());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSim =
            new SwerveDriveSimulation(
                DriveConstants.DRIVE_SIMULATION_CONFIG, new Pose2d(6, 5.5, Rotation2d.kZero));
        Arena2025Reefscape.getInstance().addDriveTrainSimulation(driveSim);
        drive =
            new Drive(
                new GyroIOSim(driveSim.getGyroSimulation()),
                new ModuleIOTalonFXSim(DriveConstants.FRONT_LEFT, driveSim.getModules()[0]),
                new ModuleIOTalonFXSim(DriveConstants.FRONT_RIGHT, driveSim.getModules()[1]),
                new ModuleIOTalonFXSim(DriveConstants.BACK_LEFT, driveSim.getModules()[2]),
                new ModuleIOTalonFXSim(DriveConstants.BACK_RIGHT, driveSim.getModules()[3]),
                driveSim::setSimulationWorldPose);
        // vision =
        //     new Vision(
        //         drive,
        //         new VisionIOPhotonVisionSim(CAM_FL_NAME, ROBOT_TO_CAM_FL_TRANSFORM,
        // drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             CAM_FR_NAME, ROBOT_TO_CAM_FR_TRANSFORM, drive::getPose));
        vision = null;
        elevator = new Elevator(new ElevatorIOSim());
        climb = new Climb(new ClimbIOSim());
        shooter = new Shooter(new ShooterIOSim());
        wrist = new Wrist(new WristIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        elevator = new Elevator(new ElevatorIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                simRobotPose -> {});
        vision = new Vision(drive, new VisionIO() {});
        climb = new Climb(new ClimbIO() {});
        shooter = new Shooter(new ShooterIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    // Create factory for commands involving moving the elevator + wrist
    elevatorAndWristCommands = new ElevatorAndWristCommands(elevator, wrist);

    // Register auto routines on dashboard selector
    Autos autos = new Autos(drive, elevator, wrist, shooter);
    autoChooser.addDefaultOption("Do Nothing", autos.doNothing());
    autoChooser.addOption("Move Forward for 2 sec", autos.moveStraight());
    autoChooser.addOption("1 Piece Center", autos.onePieceFromCenter());
    autoChooser.addOption("1 Piece Left", autos.onePieceFromLeft());
    autoChooser.addOption("1 Piece Right", autos.onePieceFromRight());
    autoChooser.addOption("1 Piece Center Blind", autos.onePieceFromCenterBlind());
    autoChooser.addOption("2 Piece Left", autos.twoPieceFromLeft());
    autoChooser.addOption("2 Piece Right", autos.twoPieceFromRight());
    autoChooser.addOption("3 Piece Left", autos.threePieceFromLeft());
    autoChooser.addOption("3 Piece Right", autos.threePieceFromRight());
    autoChooser.addOption("4 Piece Left", autos.fourPieceFromLeft());
    autoChooser.addOption("4 Piece Right", autos.fourPieceFromRight());
    autoChooser.addOption("Push + 1 Piece Center, Start at Left", autos.pushAndOnePieceFromLeft());
    autoChooser.addOption(
        "Push + 1 Piece Center, Start at Right", autos.pushAndOnePieceFromRight());

    // Configure triggers and button bindings
    configureBindings();
  }

  /**
   * Custom triggers and {@link CommandXboxController} button bindings for each subsystem are
   * defined here.
   */
  private void configureBindings() {
    /* GENERAL TRIGGERS for various robot behaviors */

    // Speed limiting based on elevator height
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.NET)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.SLOW));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.L1)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.DEFAULT));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.L2)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.SLOW));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.L3)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.SLOW));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.L4)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.SLOW));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.STATION)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.DEFAULT));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.PROCESSOR)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.DEFAULT));
    new Trigger(() -> elevator.getCurrentElevatorState() == ElevatorState.ALGAE_STOW)
        .onTrue(drive.setMaxSpeed(DriveSpeedMode.DEFAULT));

    // if running "blind" 1-piece center auton, disable vision temporarily
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> Vision.blind = false));

    // When controller disconnects, show alert
    new Trigger(() -> controller.isConnected())
        .whileFalse(Commands.runOnce(() -> controllerDisconnectedAlert.set(true)))
        .whileTrue(Commands.runOnce(() -> controllerDisconnectedAlert.set(false)));

    /* CONTROLLER BINDINGS */

    /* drive commands */
    // Default command, field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // in simulation: reset odometry to maple-sim robot pose
    // in real: zero gyro, i.e. set current heading to zero/forward (reversed on red alliance)
    final Runnable resetGyro =
        Constants.CURRENT_MODE == Constants.Mode.SIM
            ? () -> drive.setPose(driveSim.getSimulatedDriveTrainPose())
            : () -> drive.zeroGyro();

    controller.start().onTrue(drive.runOnce(resetGyro).ignoringDisable(true));

    // Auto align to nearest left/right branch and rumble after aligned
    controller
        .povLeft()
        .whileTrue(
            DriveCommands.autoAlignToNearest(drive, AutoAlignDirection.LEFT)
                .finallyDo(
                    interrupted -> {
                      if (!interrupted) {
                        rumbleCommand().schedule();
                      }
                    }));
    controller
        .povRight()
        .whileTrue(
            DriveCommands.autoAlignToNearest(drive, AutoAlignDirection.RIGHT)
                .finallyDo(
                    interrupted -> {
                      if (!interrupted) {
                        rumbleCommand().schedule();
                      }
                    }));

    // Auto align to nearest algae and rumble after aligned
    controller
        .leftStick()
        .whileTrue(
            DriveCommands.autoAlignToNearest(drive, AutoAlignDirection.ALGAE)
                .finallyDo(
                    interrupted -> {
                      if (!interrupted) {
                        rumbleCommand().schedule();
                      }
                    }));

    // Nudge forward for 0.3 seconds, with the goal of moving closer to reef
    // This was added after the edge of our shooter fractured, and coral would fall out too far away
    // from the reef.
    // This probably doesn't work as intended since it will nudge the robot forward field-relative
    // rather than robot-relative
    // However our driver was ok with this
    controller
        .rightStick()
        .onTrue(
            DriveCommands.joystickDrive(drive, () -> 0.35, () -> 0, () -> 0)
                .withTimeout(0.3)
                .finallyDo(
                    interrupted -> {
                      if (!interrupted) {
                        rumbleCommand().schedule();
                      }
                    }));

    // Toggle drive max speed
    controller.leftTrigger().onTrue(drive.toggleSpeed(elevator::getCurrentElevatorState));

    /* elevator and wrist movement commands */
    // Toggle between coral and algae mode with rumble
    controller
        .leftBumper()
        .onTrue(
            Commands.either(
                    elevatorAndWristCommands.setScoringPieceToAlgae(),
                    elevatorAndWristCommands.setScoringPieceToCoral(),
                    isCoralMode)
                .alongWith(rumbleCommand()));

    // Go to station position (thus setting robot to coral mode)
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> RobotContainer.currentScoringPieceType = ScoringPieceType.CORAL)
                .andThen(elevatorAndWristCommands.goToStation()));

    // Goes to L1 or processor based on current scoring type
    controller
        .a()
        .onTrue(
            Commands.either(
                elevatorAndWristCommands.goToL1(),
                elevatorAndWristCommands.goToProcessor(),
                isCoralMode));

    // Goes to L2 coral or algae based on current scoring type
    controller
        .b()
        .onTrue(
            Commands.either(
                elevatorAndWristCommands.goToL2Coral(),
                elevatorAndWristCommands.goToL2Algae(),
                isCoralMode));

    // Goes to L3 coral or algae based on current scoring type
    controller
        .x()
        .onTrue(
            Commands.either(
                elevatorAndWristCommands.goToL3Coral(),
                elevatorAndWristCommands.goToL3Algae(),
                isCoralMode));

    // Goes to L4 or net based on current scoring type
    controller
        .y()
        .onTrue(
            Commands.either(
                elevatorAndWristCommands.goToL4(),
                elevatorAndWristCommands.goToNet(),
                isCoralMode));

    // Goes to algae stow position, only works if in algae mode
    controller
        .povDown()
        .and(() -> currentScoringPieceType == ScoringPieceType.ALGAE)
        .onTrue(elevatorAndWristCommands.goToAlgaeStow());

    /* scoring commands */
    // Shoots coral when held and pulses shooter when released if in algae mode (holds algae better)
    controller.rightTrigger().whileTrue(shooter.shootCoralCmd(wrist::getCurrentWristState));
    controller
        .rightTrigger()
        .onFalse(Commands.either(shooter.pulseShooterCmd(), Commands.none(), isCoralMode.negate()));

    // Shoots algae; alternatively, can pull coral back into shooter
    controller.back().whileTrue(shooter.shootAlgaeCmd());

    // pulse shooter while at stow algae position
    new Trigger(() -> targetPosition == TargetState.ALGAE_STOW)
        .whileTrue(shooter.pulseShooterCmd())
        .onFalse(shooter.runOnce(() -> shooter.setTargetState(ShooterState.IDLE)));

    /* Climb commands */
    // Both of these commands only operate after 40 seconds remain in a match
    // to prevent driver from accidentally moving climb while auto-aligning with D-pad

    // Extend climb down/out. Use to latch onto the cage.
    // Only works if in coral mode,
    // because D-pad down is bound to moving to algae stow in algae mode
    // This will also set the drive to slow mode.
    controller
        .povDown()
        .and(() -> currentScoringPieceType != ScoringPieceType.ALGAE)
        .and(() -> DriverStation.getMatchTime() < 40.0)
        .whileTrue(climb.extendClimbCmd().alongWith(drive.setMaxSpeed(DriveSpeedMode.SLOW)));

    // Retract climb in/up. Use for the actual "climb" off the ground.
    controller
        .povUp()
        .and(() -> DriverStation.getMatchTime() < 40.0)
        .whileTrue(climb.retractClimbCmd());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Maple-sim simulation method. Resets the simulation field to the initial state. */
  public void resetSimulationField() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;
    Arena2025Reefscape.getInstance().resetFieldForAuto();
  }

  /**
   * Maple-sim simulation method. Updates the simulation field with the current robot pose. Logs
   * various fields.
   */
  public void updateSimulation() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

    Arena2025Reefscape.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral",
        Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae",
        Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Algae"));
  }

  /** A generic controller rumble command. Rumbles the controller for 0.2 seconds. */
  public static Command rumbleCommand() {
    return Commands.run(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 1.0);
            })
        .withTimeout(0.2)
        .finallyDo(() -> controller.setRumble(RumbleType.kBothRumble, 0.0));
  }

  /**
   * Get the current scoring piece type of the robot.
   *
   * @return
   */
  public static ScoringPieceType getCurrentScoringPieceType() {
    return currentScoringPieceType;
  }
}
