package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MOTION_MAGIC_ACCELERATION;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SPOOL_DIAMETER;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_kD;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_kG;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_kP;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_kS;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_kV;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls the elevator mechanism of the robot.
 * This is a three-stage cascade elevator, with the first stage being fixed.
 * The elevator can operate in both closed-loop and open-loop modes.
 * 
 * <p>Constructor:
 * <ul>
 *   <li>{@link #Elevator(ElevatorIO)}: Initializes the Elevator subsystem with the given IO interface.</li>
 * </ul>
 * 
 * <p>Methods:
 * <ul>
 *   <li>{@link #periodic()}: Periodically updates the elevator state, processes inputs, and handles alerts and tunable values.</li>
 *   <li>{@link #goToState(ElevatorState)}: Returns a {@link InstantCommand} to move the elevator to the specified state.</li>
 *   <li>{@link #lowerElevatorOpenLoop()}: Returns a {@link InstantCommand} to lower the elevator in open-loop mode.</li>
 *   <li>{@link #raiseElevatorOpenLoop()}: Returns a {@link InstantCommand} to raise the elevator in open-loop mode.</li>
 * </ul>
 * 
 * <p>Alerts:
 * <ul>
 *   <li>{@link #mainMotorConnectedAlert}: {@link Alert} for main elevator motor disconnection.</li>
 *   <li>{@link #followerMotorConnectedAlert}: {@link Alert} for follower elevator motor disconnection.</li>
 * </ul>
 * 
 * <p>Tunable Parameters:
 * <ul>
 *   <li>{@link #elevatorProcessorPosition}</li>
 *   <li>{@link #elevatorL1Position}</li>
 *   <li>{@link #elevatorL2Position}</li>
 *   <li>{@link #elevatorStationPosition}</li>
 *   <li>{@link #elevatorL3Position}</li>
 *   <li>{@link #elevatorL4Position}</li>
 *   <li>{@link #elevatorNetPosition}</li>
 *   <li>{@link #kP}</li>
 *   <li>{@link #kD}</li>
 *   <li>{@link #kV}</li>
 *   <li>{@link #kS}</li>
 *   <li>{@link #kG}</li>
 *   <li>{@link #vel}</li>
 *   <li>{@link #accel}</li>
 * </ul>
 */
public class Elevator extends SubsystemBase {
  /* IO and hardware inputs */
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /* Connection Alerts */
  private final Alert mainMotorConnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorConnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);

  /* State tracker for current height of the elevator */
  @AutoLogOutput private ElevatorState currentElevatorState = ElevatorState.ZERO;

  boolean isOpenLoop = false;

  LoggedTunableNumber elevatorProcessorPosition =
      new LoggedTunableNumber(
          "/Tuning/Elevator/ProcessorPosition", ElevatorState.PROCESSOR.rotations.in(Rotations));
  LoggedTunableNumber elevatorL1Position =
      new LoggedTunableNumber(
          "/Tuning/Elevator/L1Position", ElevatorState.L1.rotations.in(Rotations));
  LoggedTunableNumber elevatorL2Position =
      new LoggedTunableNumber(
          "/Tuning/Elevator/L2Position", ElevatorState.L2.rotations.in(Rotations));
  LoggedTunableNumber elevatorStationPosition =
      new LoggedTunableNumber(
          "/Tuning/Elevator/StationPosition", ElevatorState.STATION.rotations.in(Rotations));
  LoggedTunableNumber elevatorL3Position =
      new LoggedTunableNumber(
          "/Tuning/Elevator/L3Position", ElevatorState.L3.rotations.in(Rotations));
  LoggedTunableNumber elevatorL4Position =
      new LoggedTunableNumber(
          "/Tuning/Elevator/L4Position", ElevatorState.L4.rotations.in(Rotations));
  LoggedTunableNumber elevatorNetPosition =
      new LoggedTunableNumber(
          "/Tuning/Elevator/NetPosition", ElevatorState.NET.rotations.in(Rotations));

  LoggedTunableNumber kP = new LoggedTunableNumber("/Tuning/Elevator/kP", ELEVATOR_kP);
  LoggedTunableNumber kD = new LoggedTunableNumber("/Tuning/Elevator/kD", ELEVATOR_kD);
  LoggedTunableNumber kV = new LoggedTunableNumber("/Tuning/Elevator/kV", ELEVATOR_kV);
  LoggedTunableNumber kS = new LoggedTunableNumber("/Tuning/Elevator/kS", ELEVATOR_kS);
  LoggedTunableNumber kG = new LoggedTunableNumber("/Tuning/Elevator/kG", ELEVATOR_kG);

  LoggedTunableNumber vel =
      new LoggedTunableNumber("/Tuning/Elevator/vel", ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY);
  LoggedTunableNumber accel =
      new LoggedTunableNumber("/Tuning/Elevator/accel", ELEVATOR_MOTION_MAGIC_ACCELERATION);

  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  /* Periodically running code */
  @Override
  public void periodic() {
    if (!isOpenLoop) io.setTargetPosition(currentElevatorState.rotations);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double elevatorStage2HeightMeters =
        // rev * circumference/rev / gear ratio = height in meters
        Units.radiansToRotations(inputs.mainElevatorPositionRad)
            * (Math.PI * ELEVATOR_SPOOL_DIAMETER.in(Meters))
            / (ELEVATOR_GEAR_RATIO);

    double elevatorStage3HeightMeters = elevatorStage2HeightMeters * 2.0;

    Robot.componentPoses[0] = new Pose3d(0.0, 0.0, elevatorStage2HeightMeters, Rotation3d.kZero);
    Robot.componentPoses[1] = new Pose3d(0.0, 0.0, elevatorStage3HeightMeters, Rotation3d.kZero);

    if (!inputs.mainElevatorMotorConnected) {
      mainMotorConnectedAlert.set(true);
    }

    if (!inputs.followerElevatorMotorConnected) {
      followerMotorConnectedAlert.set(true);
    }

    // driver variables to visualize the elevator state
    SmartDashboard.putBoolean("Elevator HOMED", currentElevatorState == ElevatorState.ZERO);
    SmartDashboard.putBoolean("Elevator L1", currentElevatorState == ElevatorState.L1);
    SmartDashboard.putBoolean("Elevator L2", currentElevatorState == ElevatorState.L2);
    SmartDashboard.putBoolean("Elevator at STATION", currentElevatorState == ElevatorState.STATION);
    SmartDashboard.putBoolean("Elevator L3", currentElevatorState == ElevatorState.L3);
    SmartDashboard.putBoolean("Elevator L4", currentElevatorState == ElevatorState.L4);
    SmartDashboard.putBoolean("Elevator NET", currentElevatorState == ElevatorState.NET);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        nums -> {
          ElevatorState.PROCESSOR.rotations = Rotations.of(nums[0]);
          ElevatorState.L1.rotations = Rotations.of(nums[1]);
          ElevatorState.L2.rotations = Rotations.of(nums[2]);
          ElevatorState.STATION.rotations = Rotations.of(nums[3]);
          ElevatorState.L3.rotations = Rotations.of(nums[4]);
          ElevatorState.L4.rotations = Rotations.of(nums[5]);
          ElevatorState.NET.rotations = Rotations.of(nums[6]);

          if (io instanceof ElevatorIOTalonFX) {
            Slot0Configs gains =
                new Slot0Configs()
                    .withKP(nums[7])
                    .withKD(nums[8])
                    .withKV(nums[9])
                    .withKS(nums[10])
                    .withKG(nums[11]);
            MotionMagicConfigs motmag =
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(nums[12])
                    .withMotionMagicAcceleration(nums[13]);

            ((ElevatorIOTalonFX) io).mainElevatorMotor.getConfigurator().apply(gains);
            ((ElevatorIOTalonFX) io).followerElevatorMotor.getConfigurator().apply(gains);

            ((ElevatorIOTalonFX) io).mainElevatorMotor.getConfigurator().apply(motmag);
            ((ElevatorIOTalonFX) io).followerElevatorMotor.getConfigurator().apply(motmag);

            Logger.recordOutput("Elevator Configs", "Changed to: " + gains.toString());
          }
        },
        elevatorProcessorPosition,
        elevatorL1Position,
        elevatorL2Position,
        elevatorStationPosition,
        elevatorL3Position,
        elevatorL4Position,
        elevatorNetPosition,
        kP,
        kD,
        kV,
        kS,
        kG,
        vel,
        accel
        );
  }

  public Command goToState(ElevatorState targetState) {
    return this.runOnce(
        () -> {
          isOpenLoop = false;
          currentElevatorState = targetState;
        });
    }

  public Command lowerElevatorOpenLoop() {
    return this.runEnd(
        () -> {
          isOpenLoop = true;
          io.setVoltageOpenLoop(Volts.of(-0.05 * 12.0));
        },
        () -> io.setVoltageOpenLoop(Volts.zero()));
  }

  public Command raiseElevatorOpenLoop() {
    return this.runEnd(
        () -> {
          isOpenLoop = true;
          io.setVoltageOpenLoop(Volts.of(0.05 * 12.0));
        },
        () -> io.setVoltageOpenLoop(Volts.zero()));
  }
}
