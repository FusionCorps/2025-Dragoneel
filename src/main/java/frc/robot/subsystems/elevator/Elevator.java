package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert mainMotorDisconnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorDisconnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);

  /* State tracker for current height of the elevator */
  public ElevatorState currentElevatorState = ElevatorState.STATION;

  /* Triggers which track when certain points "above X position" are reached */
  @AutoLogOutput
  public Trigger isAtTargetState =
      new Trigger(
          () ->
              getCurrentElevatorPosition()
                  .isNear(Rotations.of(currentElevatorState.rotations.get()), Rotations.of(0.5)));

  @AutoLogOutput
  public Trigger isAboveL1Intermediate =
      new Trigger(
          () ->
              getCurrentElevatorPosition()
                  .gte(Rotations.of(ElevatorState.L1_INTERMEDIATE.rotations.get())));

  @AutoLogOutput
  public Trigger isAboveL3Intermediate =
      new Trigger(
          () ->
              getCurrentElevatorPosition()
                  .gte(Rotations.of(ElevatorState.L3_INTERMEDIATE.rotations.get())));

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Visualization of the elevator height
    double elevatorStage1HeightMeters =
        // rev_of_motor / gear ratio * circumference_of_drum/rev_of_drum  = height in meters
        Units.radiansToRotations(inputs.mainPositionRad)
            / ELEVATOR_GEAR_RATIO
            * (Math.PI * ELEVATOR_SPOOL_DIAMETER.in(Meters));
    double elevatorStage2HeightMeters = elevatorStage1HeightMeters * 2.0;
    Robot.componentPoses[0] = new Pose3d(0.0, 0.0, elevatorStage1HeightMeters, Rotation3d.kZero);
    Robot.componentPoses[1] = new Pose3d(0.0, 0.0, elevatorStage2HeightMeters, Rotation3d.kZero);

    if (!inputs.mainConnected) {
      mainMotorDisconnectedAlert.set(true);
    }
    if (!inputs.followerConnected) {
      followerMotorDisconnectedAlert.set(true);
    }
  }

  /** Sets the target state of the elevator. */
  public Command runTargetState(ElevatorState targetState) {
    return run(
        () -> {
          io.setTargetPosition(Rotations.of(targetState.rotations.get()));
          currentElevatorState = targetState;
        });
  }

  /**
   * Sets the elevator motors to run at "algae state" speed. I don't think this actually works since
   * this wasn't used on a CANivore and thus we cannot use DynamicMotionMagic requests.
   */
  public void setToAlgaeSpeed() {
    io.setToAlgaeSpeed();
  }

  /**
   * Sets the elevator motors to run at "coral state" speed. I don't think this actually works since
   * this wasn't used on a CANivore and thus we cannot use DynamicMotionMagic requests.
   */
  public void setToCoralSpeed() {
    io.setToCoralSpeed();
  }

  @AutoLogOutput
  public ElevatorState getCurrentElevatorState() {
    return currentElevatorState;
  }

  private Angle getCurrentElevatorPosition() {
    return Radians.of(inputs.mainPositionRad);
  }
}
