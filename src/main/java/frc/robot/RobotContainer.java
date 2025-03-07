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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ScoringPieceType;
import frc.robot.Constants.TargetState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorAndWristCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSparkFlex;
import frc.robot.util.ShootingUtil;
import java.util.Map;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
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
  private SwerveDriveSimulation driveSim = null;
  private final Vision vision;
  private final Climb climb;
  private final Shooter shooter;
  private final Wrist wrist;

  private ElevatorAndWristCommands elevatorAndWristCommands = null;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  final CommandXboxController controller = new CommandXboxController(0);
  final Alert controllerDisconnectedAlert = new Alert("Controller Disconnected.", AlertType.kError);

  private Supplier<ReefscapeCoralOnFly> simCoralProjectileSupplier = () -> null;
  @AutoLogOutput public static TargetState targetPosition = TargetState.STATION;
  @AutoLogOutput public static ScoringPieceType currentScoringPieceType = ScoringPieceType.CORAL;

  /** The container for the robot. Contains subsystems, operator devices, and commands. */
  public RobotContainer() {
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
                // (a, b, c) -> {},
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
        vision =
            new Vision(
                (a, b, c) -> {},
                // drive,
                new VisionIOPhotonVisionSim(CAM_FL_NAME, ROBOT_TO_CAM_FL_TRANSFORM, drive::getPose),
                new VisionIOPhotonVisionSim(
                    CAM_FR_NAME, ROBOT_TO_CAM_FR_TRANSFORM, drive::getPose));
        elevator = new Elevator(new ElevatorIOSim());
        climb = new Climb(new ClimbIOSim());
        shooter = new Shooter(new ShooterIOSim());
        wrist = new Wrist(new WristIOSim());
        // elevator = null;
        // climb = null;
        // shooter = null;
        // wrist = null;
        simCoralProjectileSupplier =
            () -> ShootingUtil.createCoralProjectile(drive, elevator, wrist, shooter);
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

    if (elevator != null && wrist != null) {
      elevatorAndWristCommands = new ElevatorAndWristCommands(elevator, wrist);
    }

    /* When current scoring type changes between coral and algae, elevator and wrist will toggle speed accordingly. */
    new Trigger(() -> currentScoringPieceType == ScoringPieceType.CORAL)
        .onChange(elevator.toggleElevatorSpeed().alongWith(wrist.toggleWristSpeed()));

    // Register named commands for auto
    if (drive != null && elevator != null && wrist != null && shooter != null) {
      NamedCommands.registerCommands(
          Map.of(
              "L1",
              elevatorAndWristCommands.goToL1(),
              "L2",
              elevatorAndWristCommands.goToL2Coral(),
              "Station",
              elevatorAndWristCommands.goToStation(),
              "L3",
              elevatorAndWristCommands.goToL3Coral(),
              "L4",
              elevatorAndWristCommands.goToL4(),
              "Net",
              elevatorAndWristCommands.goToNet(),
              "Processor",
              elevatorAndWristCommands.goToProcessor(),
              "ShootCoral",
              shooter.shootCoralInAutoCmd(
                  wrist.isAtScoringState,
                  elevator::getCurrentElevatorState,
                  simCoralProjectileSupplier),
              "ShootAlgae",
              shooter.shootAlgaeCmd()));
    }

    // add auto routine selector to the dashboard
    // autoChooser.addDefaultOption("Forward 2m", AutoBuilder.buildAuto("T1-Leave2M"));
    autoChooser.addDefaultOption("4 Piece Opposite Processor", AutoBuilder.buildAuto("T2-ILKJ"));
    autoChooser.addOption("1 piece bottom to E", AutoBuilder.buildAuto("B1-E"));
    autoChooser.addOption("Move straight", AutoBuilder.buildAuto("T1-Leave2M"));
    autoChooser.addOption("1 piece", AutoBuilder.buildAuto("C4-H"));
    autoChooser.addOption("wheel", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("feedforward", DriveCommands.feedforwardCharacterization(drive));

    // Set up SysId routines
    // TODO: remove these later
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

    // robot will stow when re-enabled
    // RobotModeTriggers.disabled().onTrue(elevatorAndWristCommands.goToStation());

    // When elevator leaves station, run at slower speed
    new Trigger(() -> elevator.getCurrentElevatorState() != ElevatorState.STATION)
        .onTrue(Commands.runOnce(() -> drive.setMaxSpeed(DriveSpeedMode.SLOWER)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** {@link CommandXboxController} button bindings for each subsystem are defined here. */
  private void configureButtonBindings() {
    /* drive commands */
    if (drive != null) {
      // Default command, field-relative drive
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));

      // in simulation: reset odometry to actual robot pose
      // in real: zero gyro heading
      final Runnable resetGyro =
          Constants.CURRENT_MODE == Constants.Mode.SIM
              ? () -> drive.setPose(driveSim.getSimulatedDriveTrainPose())
              : () -> drive.zeroGyro();

      controller.start().onTrue(drive.runOnce(resetGyro).ignoringDisable(true));

      // Auto align to nearest left/right branch
      controller.povLeft().whileTrue(DriveCommands.autoAlignToNearestBranch(drive, true));
      controller.povRight().whileTrue(DriveCommands.autoAlignToNearestBranch(drive, false));

      // Toggle drive speed for "slow mode" driving
      controller
          .leftTrigger()
          .onTrue(
              drive.toggleSpeed(
                  elevator::getCurrentElevatorState)); // TODO: consider making this an
      // acceleration
      // limiter, i.e. with slew rate limiting
    }

    /* elevator and wrist movement commands */
    controller
        .leftBumper()
        .onTrue(elevatorAndWristCommands.toggleScoringPieceType().andThen(rumbleCommand()));

    if (elevatorAndWristCommands != null) {
      controller
          .rightBumper()
          .onTrue(
              Commands.runOnce(
                      () -> RobotContainer.currentScoringPieceType = ScoringPieceType.CORAL)
                  .andThen(elevatorAndWristCommands.goToStation()));

      // Goes to L1 or processor based on current scoring type
      controller
          .a()
          .onTrue(
              Commands.either(
                  elevatorAndWristCommands.goToL1(),
                  elevatorAndWristCommands.goToProcessor(),
                  () -> currentScoringPieceType == ScoringPieceType.CORAL));

      // Goes to L2 coral or algae based on current scoring type
      controller
          .b()
          .onTrue(
              Commands.either(
                  elevatorAndWristCommands.goToL2Coral(),
                  elevatorAndWristCommands.goToL2Algae(),
                  () -> currentScoringPieceType == ScoringPieceType.CORAL));

      // Goes to L3 coral or algae based on current scoring type
      controller
          .x()
          .onTrue(
              Commands.either(
                  elevatorAndWristCommands.goToL3Coral(),
                  elevatorAndWristCommands.goToL3Algae(),
                  () -> currentScoringPieceType == ScoringPieceType.CORAL));

      // Goes to L4 or net based on current scoring type
      controller
          .y()
          .onTrue(
              Commands.either(
                  elevatorAndWristCommands.goToL4(),
                  elevatorAndWristCommands.goToNet(),
                  () -> currentScoringPieceType == ScoringPieceType.CORAL));

        controller.povDown().and(() -> currentScoringPieceType == ScoringPieceType.ALGAE)
            .whileTrue(elevatorAndWristCommands.goToAlgaeStow());
    }

    /* scoring commands */
    if (shooter != null && elevator != null) {
      controller
          .rightTrigger()
          .whileTrue(
              shooter.shootCoralCmd(elevator::getCurrentElevatorState, simCoralProjectileSupplier));

      controller.back().whileTrue(shooter.shootAlgaeCmd());
    }

    /* Climb commands */
    if (climb != null) {
      // extend climb out, uses composite trigger because this stows algae if in algae mode
      controller
          .povDown().and(() -> currentScoringPieceType != ScoringPieceType.ALGAE)
          .whileTrue(
                climb.extendClimbCmd()
                    .alongWith(
                        Commands.runOnce(() -> drive.setMaxSpeed(DriveSpeedMode.SLOWER))));
        
      // retract climb back in
      controller.povUp().whileTrue(climb.retractClimbCmd());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;
    Arena2025Reefscape.getInstance().resetFieldForAuto();
  }

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

  public Command rumbleCommand() {
    return Commands.run(
            () -> {
              if (currentScoringPieceType == ScoringPieceType.CORAL)
                controller.setRumble(RumbleType.kBothRumble, 0.02);
              else controller.setRumble(RumbleType.kBothRumble, 1.0);
            })
        .withTimeout(0.2)
        .finallyDo(() -> controller.setRumble(RumbleType.kBothRumble, 0.0));
  }
}
