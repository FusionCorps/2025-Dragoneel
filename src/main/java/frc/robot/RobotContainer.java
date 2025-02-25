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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorAndWristCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.module.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerIO;
import frc.robot.subsystems.scorer.ScorerIOSim;
import frc.robot.subsystems.scorer.ScorerIOSparkFlex;
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
  private final Scorer scorer;
  private final Wrist wrist;

  private ElevatorAndWristCommands elevatorAndWristCommands = null;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  private final CommandXboxController controller = new CommandXboxController(0);

  private Supplier<ReefscapeCoralOnFly> coralProjectileSupplier;

  /** The container for the robot. Contains subsystems, operator devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFXReal(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFXReal(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFXReal(DriveConstants.BACK_RIGHT),
                pose -> {});
        vision =
            new Vision(
                // drive,
                (a, b, c) -> {},
                new VisionIOPhotonVision(CAM_FL_NAME, ROBOT_TO_CAM_FL_TRANSFORM),
                new VisionIOPhotonVision(CAM_FR_NAME, ROBOT_TO_CAM_FR_TRANSFORM));
        // climb = new Climb(new ClimbIOTalonFX());
        climb = null;
        scorer = new Scorer(new ScorerIOSparkFlex());
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
        drive.setCurrentElevatorPositionSupplier(elevator::getCurrentElevatorPosition);
        // climb = new Climb(new ClimbIOSim());
        scorer = new Scorer(new ScorerIOSim());
        wrist = new Wrist(new WristIOSim());
        // elevator = null;
        climb = null;
        // scorer = null;
        // wrist = null;
        coralProjectileSupplier =
            () -> ShootingUtil.createCoralProjectile(drive, elevator, wrist, scorer);
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
                robotPose -> {});
        vision = new Vision(drive, new VisionIO() {});
        climb = new Climb(new ClimbIO() {});
        scorer = new Scorer(new ScorerIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    elevatorAndWristCommands = new ElevatorAndWristCommands(elevator, wrist);

    // elevator.isAtTargetState.onTrue(
    //     Commands.run(
    //             () -> {
    //               controller.setRumble(RumbleType.kBothRumble, 1.0);
    //             })
    //         .withTimeout(0.1)
    //         .andThen(Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))));

    // Register named commands for auto
    if (drive != null && elevator != null && wrist != null && scorer != null) {
      NamedCommands.registerCommands(
          Map.of(
              "L1", elevatorAndWristCommands.goToL1(),
              "L2", elevatorAndWristCommands.goToL2(),
              "Station", elevatorAndWristCommands.goToStation(),
              "L3", elevatorAndWristCommands.goToL3(),
              "L4", elevatorAndWristCommands.goToL4(),
              "Net", elevatorAndWristCommands.goToNet(),
              "Processor", elevatorAndWristCommands.goToProcessor(),
              "ShootCoral",
                  scorer.shootCoralCmd(elevator::getCurrentElevatorState, coralProjectileSupplier),
              "ShootAlgae", scorer.shootAlgaeCmd()));
    }

    // add auto routine selector to the dashboard
    // TODO: eventually use PathPlanner to build auto chooser
    // autoChooser.addDefaultOption("Forward 2m", AutoBuilder.buildAuto("T1-Leave2M"));
    autoChooser.addDefaultOption("4 Piece Opposite Processor", AutoBuilder.buildAuto("T2-ILKJ"));

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /** {@link CommandXboxController} button bindings for each subsystem are defined here. */
  private void configureButtonBindings() {
    /* drive commands */
    if (drive != null) {
      // Default command, normal field-relative drive
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));

      // in simulation: reset odometry to actual robot pose
      // in real: zero gyro heading
      final Runnable resetGyro =
          Constants.currentMode == Constants.Mode.SIM
              ? () -> drive.setPose(driveSim.getSimulatedDriveTrainPose())
              : () -> drive.zeroGyro();
      controller.start().onTrue(drive.zeroGyro().ignoringDisable(true));

      controller.povLeft().whileTrue(DriveCommands.autoAlignToNearestBranch(drive, true));
      controller.povRight().whileTrue(DriveCommands.autoAlignToNearestBranch(drive, false));
    }

    /* elevator and wrist movement commands */
    if (elevatorAndWristCommands != null) {
      controller.leftBumper().onTrue(elevatorAndWristCommands.goToNet());
      controller.y().onTrue(elevatorAndWristCommands.goToL4());
      controller.x().onTrue(elevatorAndWristCommands.goToL3());
      controller.b().onTrue(elevatorAndWristCommands.goToL2());
      controller.a().onTrue(elevatorAndWristCommands.goToL1());
      controller.povDown().onTrue(elevatorAndWristCommands.goToProcessor());
      controller.rightBumper().onTrue(elevatorAndWristCommands.goToStation());
    }

    if (elevator != null) {
      SmartDashboard.putData("Home Elevator Command", elevator.homeElevator());
    }

    /* scoring commands */
    if (scorer != null && elevator != null) {
      controller
          .rightTrigger()
          .whileTrue(
              scorer.shootCoralCmd(elevator::getCurrentElevatorState, coralProjectileSupplier));
      controller.leftTrigger().whileTrue(scorer.shootAlgaeCmd());
    }

    if (climb != null) {
      // run climb
      controller.povUp().whileTrue(climb.runClimbCmd());
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
    if (Constants.currentMode != Constants.Mode.SIM) return;
    Arena2025Reefscape.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Arena2025Reefscape.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral",
        Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae",
        Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
