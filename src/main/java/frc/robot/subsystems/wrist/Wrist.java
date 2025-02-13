package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.WristState;
import frc.robot.Robot;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  @AutoLogOutput private WristState currentWristState = WristState.ZERO;

  private final Alert wristMotorConnectedAlert =
      new Alert("Wrist Motor Disconnected.", AlertType.kError);

  LoggedTunableNumber wristProcessorPosition =
      new LoggedTunableNumber("/Tuning/Wrist/ProcessorPosition", 0.0);
  LoggedTunableNumber wristL1Position = new LoggedTunableNumber("/Tuning/Wrist/L1Position", 0.0);
  LoggedTunableNumber wristL2_AND_L3Position =
      new LoggedTunableNumber("/Tuning/Wrist/L2_AND_L3Position", 0.0);
  LoggedTunableNumber wristStationPosition =
      new LoggedTunableNumber("/Tuning/Wrist/StationPosition", 0.0);
  LoggedTunableNumber wristL4Position = new LoggedTunableNumber("/Tuning/Wrist/L4Position", 0.0);
  LoggedTunableNumber wristNetPosition = new LoggedTunableNumber("/Tuning/Wrist/NetPosition", 0.0);

  LoggedTunableNumber wristkP = new LoggedTunableNumber("/Tuning/Wrist/kP", 0.0);
  LoggedTunableNumber wristkD = new LoggedTunableNumber("/Tuning/Wrist/kD", 0.0);

  boolean isOpenLoop = false;

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (!isOpenLoop) io.setTargetPosition(currentWristState.rotations);
    io.updateInputs(inputs);

    Robot.componentPoses[2] =
        new Pose3d(
            0.14,
            -0.04,
            Robot.componentPoses[1].getZ() + 0.55,
            new Rotation3d(0, inputs.wristPositionRad, 0));

    Logger.processInputs("Wrist", inputs);
    wristMotorConnectedAlert.set(!inputs.wristMotorConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        nums -> {
          WristState.L1.rotations = Rotations.of(nums[0]);
          WristState.L2_AND_L3.rotations = Rotations.of(nums[1]);
          WristState.L4.rotations = Rotations.of(nums[2]);
          WristState.PROCESSOR.rotations = Rotations.of(nums[3]);
          WristState.STATION.rotations = Rotations.of(nums[4]);
          WristState.NET.rotations = Rotations.of(nums[5]);

          if (io instanceof WristIOSparkFlex) {
            SparkFlex sparkFlex = ((WristIOSparkFlex) io).wristMotor;
            sparkFlex.configureAsync(
                new SparkFlexConfig().apply(new ClosedLoopConfig().p(nums[6])),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
            sparkFlex.configureAsync(
                new SparkFlexConfig().apply(new ClosedLoopConfig().d(nums[7])),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
          }
        },
        wristProcessorPosition,
        wristL1Position,
        wristL2_AND_L3Position,
        wristStationPosition,
        wristL4Position,
        wristNetPosition,
        wristkP,
        wristkD);
  }

  public Command moveWristRight() {
    return this.runEnd(
        () -> {
          io.setVoltageOpenLoop(Volts.of(0.01 * 12.0));
          isOpenLoop = true;
        },
        () -> io.setVoltageOpenLoop(Volts.zero()));
  }

  public Command moveWristLeft() {
    return this.runEnd(
        () -> {
          io.setVoltageOpenLoop(Volts.of(-0.01 * 12.0));
          isOpenLoop = true;
        },
        () -> io.setVoltageOpenLoop(Volts.zero()));
  }

  public Command goToState(WristState state) {
    return this.runOnce(
        () -> {
          currentWristState = state;
          isOpenLoop = false;
        });
  }
}
