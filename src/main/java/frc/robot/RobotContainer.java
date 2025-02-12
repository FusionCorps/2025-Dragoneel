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

import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerIO;
import frc.robot.subsystems.scorer.ScorerIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import java.util.Map;
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
  private final Vision vision;
  private final Climb climb;
  private final Scorer scorer;
  private final Wrist wrist;

  private final LoggedDashboardChooser<Command> autoChooser;
  SuperstructureCommands superstructureCommands;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
        // drive = null;
        // vision =
        //     new Vision(
        //         drive,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
        // vision = new Vision((a, b, c) -> {}, new VisionIOPhotonVision(camera0Name,
        // robotToCamera0));
        vision = null;

        climb = null;
        // climb = new Climb(new ClimbIOTalonFX());
        scorer = null;
        // scorer = new Scorer(new ScorerIOSparkFlex());
        elevator = new Elevator(new ElevatorIOTalonFX());
        // elevator = null;

        // wrist = new Wrist(new WristIOSparkFlex());
        wrist = null;
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSim());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FRONT_LEFT),
                new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                new ModuleIOSim(DriveConstants.BACK_LEFT),
                new ModuleIOSim(DriveConstants.BACK_RIGHT));
        vision =
            new Vision(
                (a, b, c) -> {},
                // drive,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        climb = new Climb(new ClimbIOSim());
        scorer = new Scorer(new ScorerIOSim());
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
                new ModuleIO() {});
        vision = new Vision(drive, new VisionIO() {});
        climb = new Climb(new ClimbIO() {});
        scorer = new Scorer(new ScorerIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    // Set up auto routines
    // register commands for PathPlanner

    if (elevator != null && wrist != null)
      superstructureCommands = new SuperstructureCommands(elevator, wrist);

    if (elevator != null && scorer != null && climb != null) {
      NamedCommands.registerCommands(
          Map.of(
              "L1", superstructureCommands.goToL1(),
              "L2", superstructureCommands.goToL2(),
              "Station", superstructureCommands.goToStation(),
              "L3", superstructureCommands.goToL3(),
              "L4", superstructureCommands.goToL4(),
              "Net", superstructureCommands.goToNet(),
              "Processor", superstructureCommands.goToProcessor(),
              "ShootCoral", scorer.shootCoralCmd(),
              "ShootAlgae", scorer.shootAlgaeCmd()));
    }

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    // autoChooser.addDefaultOption("Forward 2m", AutoBuilder.buildAuto("T1-Leave2M"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    if (drive != null) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX()));

      // Reset gyro to 0° when B button is pressed
      controller.start().onTrue(drive.zeroOdometry());

      // controller
      //     .povRight()
      //     .whileTrue(DriveCommands.driveToNearestReefTagWOdometryAndOffset(drive, false));

      // controller
      //     .povLeft()
      //     .whileTrue(DriveCommands.driveToNearestReefTagWOdometryAndOffset(drive, true));
    }

    if (elevator != null && wrist != null) {
      controller.leftBumper().onTrue(superstructureCommands.goToNet());
      controller.y().onTrue(superstructureCommands.goToL4());
      controller.x().onTrue(superstructureCommands.goToL3());
      controller.b().onTrue(superstructureCommands.goToL2());
      controller.a().onTrue(superstructureCommands.goToL1());
      controller.povDown().onTrue(superstructureCommands.goToProcessor());
      controller.rightBumper().onTrue(superstructureCommands.goToStation());
    }

    // TODO: eventually remove this block in favor of supersructure commands
    if (elevator != null) {
      controller.leftBumper().onTrue(elevator.goToNet());
      controller.y().onTrue(elevator.goToL4());
      controller.x().onTrue(elevator.goToL3());
      controller.b().onTrue(elevator.goToL2());
      controller.a().onTrue(elevator.goToL1());
      controller.povDown().onTrue(elevator.goToZero());
      controller.rightBumper().onTrue(elevator.goToStation());

      // controller.back().whileTrue(elevator.runHomingRoutine());

      // TODO: remove after closed loop control is tuned
      // controller.rightTrigger().whileTrue(elevator.lowerElevator());
      // controller.leftTrigger().whileTrue(elevator.raiseElevator());
    }

    if (scorer != null) {
      controller.rightTrigger().whileTrue(scorer.shootCoralCmd());
      controller.leftTrigger().whileTrue(scorer.shootAlgaeCmd());
    }

    if (climb != null) {
      controller.povUp().whileTrue(climb.runClimbCmd());
    }

    if (wrist != null) {
      // TODO: remove after closed loop control is tuned
      controller.povRight().whileTrue(wrist.moveWristRight());
      controller.povLeft().whileTrue(wrist.moveWristLeft());
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
}
