package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class DriveConstants {
  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(90)
          .withKI(0)
          .withKD(0.1)
          .withKS(0.0)
          .withKV(1.5)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  public static final Slot0Configs driveGains =
      new Slot0Configs()
          .withKP(0.2)
          .withKI(0)
          .withKD(0)
          .withKS(0.18271)
          .withKV(0.82)
          .withKA(0.0055232);
  ;

  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the drive motor
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // The stator current at which the wheels start to slip
  private static final Current kSlipCurrent = Amps.of(120.0);

  // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively
                  // low stator current limit to help avoid brownouts without impacting
                  // performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null
  private static final Pigeon2Configuration pigeonConfigs = null;

  // CAN bus that the devices are located on;
  // All swerve devices must share the same CAN bus
  public static final CANBus kCANBus = new CANBus("canivore");

  // Theoretical free speed (m/s) at 12 V applied output
  public static LinearVelocity SPEED_AT_12V = MetersPerSecond.of(4.32816);
  public static AngularVelocity MODULE_ANGULAR_VEL_AT_12V = RotationsPerSecond.of(60);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
  private static final double kCoupleRatio = 3;

  // Gear ratios for the drive and steer motors
  private static final double kDriveGearRatio = 7.125;
  private static final double kSteerGearRatio = 18.75;

  // Wheel radius
  private static final Distance kWheelRadius = Inches.of(2);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final int kPigeonId = 0;

  // These constants are only used for simulation
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  // Simulated voltage necessary to overcome friction
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(SPEED_AT_12V)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 3;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.48291015625);
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(10.375);
  private static final Distance kFrontLeftYPos = Inches.of(10.375);

  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 5;
  private static final int kFrontRightEncoderId = 6;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(0.3681640625);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10.375);
  private static final Distance kFrontRightYPos = Inches.of(-10.375);

  // Back Left
  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 8;
  private static final int kBackLeftEncoderId = 9;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(0.07763671875);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-10.375);
  private static final Distance kBackLeftYPos = Inches.of(10.375);

  // Back Right
  private static final int kBackRightDriveMotorId = 10;
  private static final int kBackRightSteerMotorId = 11;
  private static final int kBackRightEncoderId = 12;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.07958984375);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-10.375);
  private static final Distance kBackRightYPos = Inches.of(-10.375);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);

  public static Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(FRONT_LEFT.LocationX, FRONT_LEFT.LocationY),
        new Translation2d(FRONT_RIGHT.LocationX, FRONT_RIGHT.LocationY),
        new Translation2d(BACK_LEFT.LocationX, BACK_LEFT.LocationY),
        new Translation2d(BACK_RIGHT.LocationX, BACK_RIGHT.LocationY)
      };

  // wheel coefficient of friction (guesstimate) for GripLock V2 wheels
  public static final double WHEEL_COF = 1.5;

  // TunerConstants doesn't include these constants, so they are declared locally
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(DRIVETRAIN_CONSTANTS.CANBusName).isNetworkFD() ? 250.0 : 100.0;

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(FRONT_LEFT.LocationX, FRONT_LEFT.LocationY),
              Math.hypot(FRONT_RIGHT.LocationX, FRONT_RIGHT.LocationY)),
          Math.max(
              Math.hypot(BACK_LEFT.LocationX, BACK_LEFT.LocationY),
              Math.hypot(BACK_RIGHT.LocationX, BACK_RIGHT.LocationY)));

  // maple-sim config
  public static DriveTrainSimulationConfig DRIVE_SIMULATION_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withRobotMass(Constants.ROBOT_MASS)
          .withSwerveModule(
              COTS.ofMark4n(DCMotor.getKrakenX60Foc(1), DCMotor.getFalcon500Foc(1), 1.9, 1));

  // max speeds enums
  public static enum DriveSpeedMode {
    DEFAULT(List.of(SPEED_AT_12V.in(MetersPerSecond), Units.rotationsToRadians(1.0))),
    SLOW(List.of(1.0, Units.rotationsToRadians(0.5))),
    PRECISION(List.of(0.5, Units.rotationsToRadians(0.35)));

    List<Double> speedAndTheta;

    DriveSpeedMode(List<Double> speedAndTheta) {
      this.speedAndTheta = speedAndTheta;
    }
  }

  /** Robot-relative auto align directions */
  public static enum AutoAlignDirection {
    LEFT,
    RIGHT,
    ALGAE,
    BARGE
  }

  // auto-align offset baselines
  // more negative is CLOSER to the reef
  // more positive is FURTHER AWAY from the reef
  public static double autoAlignOutCoralLeftBaseline = 0.585;
  public static double autoAlignOutCoralRightBaseline = 0.604;

  public static double autoAlignSideCoralLeftBaseline = -0.362;
  public static double autoAlignSideCoralRightBaseline = -0.056;

  public static double autoAlignOutAlgaeBaseline = 0.585;
  public static double autoAlignSideAlgaeBaseline = -0.08;

  public static double autoAlignOutBargeBaseline = 0.585;
  public static double autoAlignSideBargeBaseline = 0.00;

  public static LoggedTunableNumber autoAlignOutCoralLeft =
      new LoggedTunableNumber("AutoAlign/outLeft", 0);
  public static LoggedTunableNumber autoAlignOutCoralRight =
      new LoggedTunableNumber("AutoAlign/outRight", 0);

  // more negative is to the left OF THE ROBOT
  // more positive is to the right OF THE ROBOT
  public static LoggedTunableNumber autoAlignSideCoralLeft =
      new LoggedTunableNumber("AutoAlign/sideLeft", 0);
  public static LoggedTunableNumber autoAlignSideCoralRight =
      new LoggedTunableNumber("AutoAlign/sideRight", 0);

  public static LoggedTunableNumber autoAlignOutAlgae =
      new LoggedTunableNumber("AutoAlign/outAlgae", 0);
  public static LoggedTunableNumber autoAlignSideAlgae =
      new LoggedTunableNumber("AutoAlign/sideAlgae", 0);

  public static LoggedTunableNumber autoAlignOutBarge =
      new LoggedTunableNumber("AutoAlign/outBarge", 0);
  public static LoggedTunableNumber autoAlignSideBarge =
      new LoggedTunableNumber("AutoAlign/sideBarge", 0);
}
