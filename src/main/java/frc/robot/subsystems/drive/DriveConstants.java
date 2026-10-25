package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class DriveConstants {
  public static final CANBus kCANBus = new CANBus("canivore");

  // 50 hz temporary fix to prevent command overruns.
  // TODO: investigate optimal frequency for RoboRIO 1.
  public static final Frequency kStatusFrequency = Hertz.of(50.0);
  public static final Frequency kOdometryFrequency = Hertz.of(kCANBus.isNetworkFD() ? 50.0 : 50.0);

  public static final FeedbackSensorSourceValue kFeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
  public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  private static final Slot0Configs SteerGains = new Slot0Configs()
      .withKP(33)
      .withKI(0)
      .withKD(0.5)
      .withKS(0.1)
      .withKV(2.49)
      .withKA(0)
      .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs DriveGains = new Slot0Configs()
    .withKP(0.1).withKI(0).withKD(0)
    .withKS(0).withKV(0.747);

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  private static final TalonFXConfiguration DriveInitialConfigs = new TalonFXConfiguration()
    .withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(40))
        .withStatorCurrentLimitEnable(true)
    );
  private static final TalonFXConfiguration SteerInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(Amps.of(40))
              .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration EncoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration Pigeon2InitialConfigs = new Pigeon2Configuration();

  private static final Current kSlipCurrent = Amps.of(120.0);
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.25);
  public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);

  private static final double kCoupleRatio = 3.857142857142857;
  private static final double kDriveGearRatio = 6.026785714285714;
  private static final double kSteerGearRatio = 26.09090909090909;

  private static final Distance kWheelRadius = Inches.of(2.05);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  public static final int kPigeonId = 12;

  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.001);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.035);
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(kCANBus.getName())
      .withPigeon2Id(kPigeonId)
      .withPigeon2Configs(Pigeon2InitialConfigs);

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
      .withDriveMotorGearRatio(kDriveGearRatio)
      .withSteerMotorGearRatio(kSteerGearRatio)
      .withCouplingGearRatio(kCoupleRatio)
      .withWheelRadius(kWheelRadius)
      .withSteerMotorGains(SteerGains)
      .withDriveMotorGains(DriveGains)
      .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
      .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
      .withSlipCurrent(kSlipCurrent)
      .withSpeedAt12Volts(kSpeedAt12Volts)
      .withDriveMotorType(kDriveMotorType)
      .withSteerMotorType(kSteerMotorType)
      .withFeedbackSource(kSteerFeedbackType)
      .withDriveMotorInitialConfigs(DriveInitialConfigs)
      .withSteerMotorInitialConfigs(SteerInitialConfigs)
      .withEncoderInitialConfigs(EncoderInitialConfigs)
      .withSteerInertia(kSteerInertia)
      .withDriveInertia(kDriveInertia)
      .withSteerFrictionVoltage(kSteerFrictionVoltage)
      .withDriveFrictionVoltage(kDriveFrictionVoltage);

  private static final int kFrontLeftDriveMotorId = 9;
  private static final int kFrontLeftSteerMotorId = 11;
  private static final int kFrontLeftEncoderId = 10;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.297607421875);
  private static final boolean kFrontLeftSteerMotorInverted = false;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(10.945);
  private static final Distance kFrontLeftYPos = Inches.of(10.945);

  private static final int kFrontRightDriveMotorId = 6;
  private static final int kFrontRightSteerMotorId = 8;
  private static final int kFrontRightEncoderId = 7;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.43798828125);
  private static final boolean kFrontRightSteerMotorInverted = false;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10.945);
  private static final Distance kFrontRightYPos = Inches.of(-10.945);

  private static final int kBackLeftDriveMotorId = 0;
  private static final int kBackLeftSteerMotorId = 2;
  private static final int kBackLeftEncoderId = 1;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.31298828125);
  private static final boolean kBackLeftSteerMotorInverted = false;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-10.945);
  private static final Distance kBackLeftYPos = Inches.of(10.945);

  private static final int kBackRightDriveMotorId = 3;
  private static final int kBackRightSteerMotorId = 5;
  private static final int kBackRightEncoderId = 4;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.01171875);
  private static final boolean kBackRightSteerMotorInverted = false;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-10.945);
  private static final Distance kBackRightYPos = Inches.of(-10.945);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
      .createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          kFrontLeftXPos,
          kFrontLeftYPos,
          kInvertLeftSide,
          kFrontLeftSteerMotorInverted,
          kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
      .createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          kFrontRightXPos,
          kFrontRightYPos,
          kInvertRightSide,
          kFrontRightSteerMotorInverted,
          kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
      .createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          kBackLeftXPos,
          kBackLeftYPos,
          kInvertLeftSide,
          kBackLeftSteerMotorInverted,
          kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
      .createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          kBackRightXPos,
          kBackRightYPos,
          kInvertRightSide,
          kBackRightSteerMotorInverted,
          kBackRightEncoderInverted);

  public static final double kDriveBaseRadius = Math.max(
      Math.max(
          Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
          Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
      Math.max(
          Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
          Math.hypot(BackRight.LocationX, BackRight.LocationY)));

  public static final Time kDeltaTime = Seconds.of(0.02);

  public static final double kRobotMassKg = 68.04;
  public static final double kRobotMOI = 6.883;
  public static final double kWheelCof = 1.2;

  public static final Angle kAngleTolerance = Degrees.of(3.0);

  public static final Translation2d[] kModuleTranslations = {
      new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
      new Translation2d(FrontRight.LocationX, FrontRight.LocationY),
      new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
      new Translation2d(BackRight.LocationX, BackRight.LocationY)
  };

  public static class PathPlanner {
    public static final PIDConstants TranslationGains = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants RotationGains = new PIDConstants(5.0, 0.0, 0.0);

    public static final RobotConfig Config = new RobotConfig(
        kRobotMassKg,
        kRobotMOI,
        new ModuleConfig(
            FrontLeft.WheelRadius,
            kSpeedAt12Volts.in(MetersPerSecond),
            kWheelCof,
            DCMotor.getKrakenX60Foc(1)
                .withReduction(FrontLeft.DriveMotorGearRatio),
            FrontLeft.SlipCurrent,
            1),
        kModuleTranslations);
  }

  public static class Simulation {
    public static class DriveGains {
      public static final double kP = 0.05;
      public static final double kD = 0.0;
      public static final double kS = 0.0;
      public static final double kVRotation = 0.91035;
      public static final double kV = 1.0 / Units.rotationsToRadians(1.0 / kVRotation);
    }

    public static class SteerGains {
      public static final double kP = 8.0;
      public static final double kD = 0.0;
    }

    public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(1);
    public static final DCMotor kSteerMotor = DCMotor.getKrakenX44(1);
  }
}
