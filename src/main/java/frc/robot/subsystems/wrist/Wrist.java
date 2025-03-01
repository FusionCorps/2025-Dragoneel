package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  @AutoLogOutput private WristState currentWristState = WristState.STATION;

  @AutoLogOutput
  public Trigger isAtStation =
      new Trigger(() -> WristState.STATION.rotations.isNear(getCurrentAngle(), Degrees.of(15)));

  @AutoLogOutput
  public Trigger isAtScoringState =
      new Trigger(
          () ->
              currentWristState.rotations.isNear(getCurrentAngle(), Degrees.of(5))
                  && currentWristState != WristState.STATION);

  private final Alert wristMotorDisconnectedAlert =
      new Alert("Wrist Motor Disconnected.", AlertType.kError);

  LoggedTunableNumber wristProcessorPosition =
      new LoggedTunableNumber(
          "/Wrist/ProcessorPosition", WristState.PROCESSOR.rotations.in(Rotations));
  LoggedTunableNumber wristL1Position =
      new LoggedTunableNumber("/Wrist/L1Position", WristState.L1.rotations.in(Rotations));
  LoggedTunableNumber wristL2Position =
      new LoggedTunableNumber("/Wrist/L2Position", WristState.L2_CORAL.rotations.in(Rotations));
  LoggedTunableNumber wristL3Position =
      new LoggedTunableNumber("/Wrist/L3Position", WristState.L3_CORAL.rotations.in(Rotations));
  LoggedTunableNumber wristL4Position =
      new LoggedTunableNumber("/Wrist/L4Position", WristState.L4.rotations.in(Rotations));
  LoggedTunableNumber wristNetPosition =
      new LoggedTunableNumber("/Wrist/NetPosition", WristState.NET.rotations.in(Rotations));

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.setTargetPosition(currentWristState.rotations);
    io.updateInputs(inputs);

    Robot.componentPoses[2] =
        new Pose3d(
            0.135,
            -0.06,
            Robot.componentPoses[1].getZ() + 0.525,
            new Rotation3d(0, -inputs.absolutePositionRad, 0));

    Logger.processInputs("Wrist", inputs);
    wristMotorDisconnectedAlert.set(!inputs.connected);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        nums -> {
          WristState.PROCESSOR.rotations = Rotations.of(nums[0]);
          WristState.L1.rotations = Rotations.of(nums[1]);
          WristState.L2_CORAL.rotations = Rotations.of(nums[2]);
          WristState.L3_CORAL.rotations = Rotations.of(nums[3]);
          WristState.L4.rotations = Rotations.of(nums[4]);
          WristState.NET.rotations = Rotations.of(nums[5]);
        },
        wristProcessorPosition,
        wristL1Position,
        wristL2Position,
        wristL3Position,
        wristL4Position,
        wristNetPosition);
  }

  public Command goToState(WristState state) {
    return this.runOnce(
        () -> {
          currentWristState = state;
        });
  }

  public Angle getCurrentAngle() {
    return Radians.of(inputs.absolutePositionRad);
  }
}
