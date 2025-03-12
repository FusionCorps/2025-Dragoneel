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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.util.ShootingUtil;
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
            // new Vision(
            //     (a, b, c) -> {},
            //     // drive,
            //     new VisionIOPhotonVisionSim(CAM_FL_NAME, ROBOT_TO_CAM_FL_TRANSFORM,
            // drive::getPose),
            //     new VisionIOPhotonVisionSim(
            //         CAM_FR_NAME, ROBOT_TO_CAM_FR_TRANSFORM, drive::getPose));
            null;
        elevator = new Elevator(new ElevatorIOSim());
        climb = new Climb(new ClimbIOSim());
        shooter = new Shooter(new ShooterIOSim());
        wrist = new Wrist(new WristIOSim());

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

    // Register auto routines on dashboard chooser
    if (drive != null && elevator != null && wrist != null && shooter != null) {
      Autos autos = new Autos(drive, elevator, wrist, shooter);
      autoChooser.addDefaultOption("Do Nothing", autos.doNothing());
      autoChooser.addOption("Move Forward for 2 sec", autos.moveStraight());
      autoChooser.addOption("1 Piece Center", autos.onePieceFromCenter());
      autoChooser.addOption("1 Piece Top", autos.onePieceFromTop());
      autoChooser.addOption("1 Piece Bottom", autos.onePieceFromBottom());
      autoChooser.addOption("2 Piece Top", autos.twoPieceFromTop());
      autoChooser.addOption("2 Piece Bottom", autos.twoPieceFromBottom());
      autoChooser.addOption("3 Piece Top", autos.threePieceFromTop());
      autoChooser.addOption("3 Piece Bottom", autos.threePieceFromBottom());
      autoChooser.addOption("4 Piece Top", autos.fourPieceFromTop());
      autoChooser.addOption("4 Piece Bottom", autos.fourPieceFromBottom());
      autoChooser.addOption("Push + 1 Piece Center, Start at Top", autos.pushAndOnePieceFromTop());
      autoChooser.addOption(
          "Push + 1 Piece Center, Start at Bottom", autos.pushAndOnePieceFromBottom());
    }

    // add diagnostic and sysid commands
    // TODO: remove these later

    autoChooser.addOption("wheel", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("feedforward", DriveCommands.feedforwardCharacterization(drive));

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

    // When elevator leaves station, run at slower speed
    new Trigger(() -> elevator.getCurrentElevatorState() != ElevatorState.STATION)
        .onTrue(Commands.runOnce(() -> drive.setMaxSpeed(DriveSpeedMode.SLOWER)));

    // When elevator changes scoring piece type, rumble controller
    new Trigger(() -> currentScoringPieceType == ScoringPieceType.CORAL).onChange(rumbleCommand());

    // When controller disconnects, show alert
    new Trigger(() -> controller.isConnected())
        .whileFalse(Commands.runOnce(() -> controllerDisconnectedAlert.set(true)))
        .whileTrue(Commands.runOnce(() -> controllerDisconnectedAlert.set(false)));

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
      controller
          .povLeft()
          .whileTrue(DriveCommands.autoAlignToNearestBranch(drive, AutoAlignDirection.LEFT));
      controller
          .povRight()
          .whileTrue(DriveCommands.autoAlignToNearestBranch(drive, AutoAlignDirection.RIGHT));

      // Toggle drive speed for "slow mode" driving
      controller.leftTrigger().onTrue(drive.toggleSpeed(elevator::getCurrentElevatorState));
      // TODO: consider making this an
      // acceleration
      // limiter, i.e. with slew rate limiting
    }

    /* elevator and wrist movement commands */
    controller.leftBumper().onTrue(elevatorAndWristCommands.setScoringPieceToAlgae());

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

      controller
          .povDown()
          .and(() -> currentScoringPieceType == ScoringPieceType.ALGAE)
          .whileTrue(elevatorAndWristCommands.goToAlgaeStow());
    }

    /* scoring commands */
    if (shooter != null && elevator != null) {
      controller
          .rightTrigger()
          .whileTrue(
              shooter.shootCoralCmd(wrist::getCurrentWristState, simCoralProjectileSupplier));

      controller.back().whileTrue(shooter.shootAlgaeCmd());
    }

    /* Climb commands */
    if (climb != null) {
      // extend climb out, uses composite trigger because this stows algae if in algae mode
      controller
          .povDown()
          .and(() -> currentScoringPieceType != ScoringPieceType.ALGAE)
          .whileTrue(
              climb
                  .extendClimbCmd()
                  .alongWith(Commands.runOnce(() -> drive.setMaxSpeed(DriveSpeedMode.SLOWER))));

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
