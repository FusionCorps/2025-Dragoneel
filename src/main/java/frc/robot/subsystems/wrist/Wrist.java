package frc.robot.subsystems.wrist;

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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  @AutoLogOutput private WristState currentWristState = WristState.ZERO;

  private final Alert wristMotorConnectedAlert =
      new Alert("Wrist Motor Disconnected.", AlertType.kError);

  LoggedNetworkNumber wristProcessorPosition =
      new LoggedNetworkNumber("/Tuning/Wrist/ProcessorPosition", 0.0);
  LoggedNetworkNumber wristL1Position = new LoggedNetworkNumber("/Tuning/Wrist/L1Position", 0.0);
  LoggedNetworkNumber wristL2_AND_L3Position =
      new LoggedNetworkNumber("/Tuning/Wrist/L2_AND_L3Position", 0.0);
  LoggedNetworkNumber wristStationPosition =
      new LoggedNetworkNumber("/Tuning/Wrist/StationPosition", 0.0);
  LoggedNetworkNumber wristL4Position = new LoggedNetworkNumber("/Tuning/Wrist/L4Position", 0.0);
  LoggedNetworkNumber wristNetPosition = new LoggedNetworkNumber("/Tuning/Wrist/NetPosition", 0.0);

  LoggedNetworkNumber wristkP = new LoggedNetworkNumber("/Tuning/Wrist/kP", 0.0);
  LoggedNetworkNumber wristkD = new LoggedNetworkNumber("/Tuning/Wrist/kD", 0.0);

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

    // WristState.L1.rotations = Rotations.of(wristL1Position.get());
    // WristState.L2_AND_L3.rotations = Rotations.of(wristL2_AND_L3Position.get());
    // WristState.L4.rotations = Rotations.of(wristL4Position.get());
    // WristState.NET.rotations = Rotations.of(wristNetPosition.get());
    // WristState.PROCESSOR.rotations = Rotations.of(wristProcessorPosition.get());
    // WristState.STATION.rotations = Rotations.of(wristStationPosition.get());

    if (io instanceof WristIOSparkFlex) {
      SparkFlex sparkFlex = ((WristIOSparkFlex) io).wristMotor;
      double cachedkP = sparkFlex.configAccessor.closedLoop.getP();
      double cachedkD = sparkFlex.configAccessor.closedLoop.getD();
      if (cachedkP != wristkP.get()) {
        sparkFlex.configureAsync(
            new SparkFlexConfig().apply(new ClosedLoopConfig().p(wristkP.get())),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
      }
      if (cachedkD != wristkD.get()) {
        sparkFlex.configureAsync(
            new SparkFlexConfig().apply(new ClosedLoopConfig().d(wristkD.get())),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
      }
    }
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

  public Command goToZero() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.ZERO;
          isOpenLoop = false;
        });
  }

  public Command goToProcessor() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.PROCESSOR;
          isOpenLoop = false;
        });
  }

  public Command goToL1() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.L1;
          isOpenLoop = false;
        });
  }

  public Command goToL2() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.L2_AND_L3;
          isOpenLoop = false;
        });
  }

  public Command goToL3() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.L2_AND_L3;
          isOpenLoop = false;
        });
  }

  public Command goToStation() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.STATION;
          isOpenLoop = false;
        });
  }

  public Command goToL4() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.L4;
          isOpenLoop = false;
        });
  }

  public Command goToNet() {
    return this.runOnce(
        () -> {
          currentWristState = WristState.NET;
          isOpenLoop = false;
        });
  }
}
