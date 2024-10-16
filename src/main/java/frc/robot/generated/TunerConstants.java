package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.lib.swerve.Swerve;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs().withKP(7.75345).withKI(0).withKD(0).withKS(0.19368).withKV(0.1228675).withKA(0.01380675);


    //The closed loop output type to use for steer motors
    //Affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType steerClosedLoopOutputType = ClosedLoopOutputType.Voltage;

    //The closed loop output type to use for drive motors
    //Affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType driveClosedLoopOutputType = ClosedLoopOutputType.Voltage;

    //the stator current at which the wheels start to slip
    //needs to be tuned to your individual robot
    public static final double kSlipCurrentA = 300.0;


    //the theoretical speed of your robot when supplied 12 volts
    //must be tuned to specific robot
    public static final double kSpeedAt12VoltsMps = 4.48;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double kCoupleRatio = 3.5714285714285716;

    public static final double kDriveGearRatio = 6.746031746031747;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final double kWheelRadiusInches = 1.895;

    public static final boolean kSteerMotorReversed = true;
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final String kCANbusName = "";
    public static final int kPigeonId = 0;

    // These are only used for simulation
    public static final double kSteerInertia = 0.00001;
    public static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    public static final double kSteerFrictionVoltage = 0.25;
    public static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);
    
    public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(kDriveGearRatio)
        .withSteerMotorGearRatio(kSteerGearRatio)
        .withWheelRadius(kWheelRadiusInches)
        .withSlipCurrent(kSlipCurrentA)
        .withSteerMotorGains(steerGains)
        .withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(steerClosedLoopOutputType)
        .withDriveMotorClosedLoopOutput(driveClosedLoopOutputType)
        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
        .withSteerInertia(kSteerInertia)
        .withDriveInertia(kDriveInertia)
        .withSteerFrictionVoltage(kSteerFrictionVoltage)
        .withDriveFrictionVoltage(kDriveFrictionVoltage)
        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
        .withCouplingGearRatio(kCoupleRatio)
        .withSteerMotorInverted(kSteerMotorReversed);

    //Front left
    public static final int kFrontLeftDriveMotorId = 0;
    public static final int kFrontLeftSteerMotorId = 1;
    public static final int kFrontLeftEncoderId = 0;
    public static final double kFrontLeftEncoderOffset = 0.25244140625;

    public static final double kFrontLeftXPosInches = 12;
    public static final double kFrontLeftYPosInches = 12;

    //Front right
    public static final int kFrontRightDriveMotorId = 2;
    public static final int kFrontRightSteerMotorId = 3;
    public static final int kFrontRightEncoderId =1;
    public static final double kFrontRightEncoderOffset = -0.329345703125;

    public static final double kFrontRightXPosInches = 12;
    public static final double kFrontRightYPosInches = -12;

    //Back left
    public static final int kBackLeftDriveMotorId = 4;
    public static final int kBackLeftSteerMotorId = 5;
    public static final int kBackLeftEncoderId = 2;
    public static final double kBackLeftEncoderOffset = 0.187255859375;

    public static final double kBackLeftXPosInches = -12;
    public static final double kBackLeftYPosInches = 12;

    //Back right
    public static final int kBackRightDriveMotorId = 6;
    public static final int kBackRightSteerMotorId = 7;
    public static final int kBackRightEncoderId = 3;
    public static final double kBackRightEncoderOffset = -0.4951171875;

    public static final double kBackRightXPosInches = -12;
    public static final double kBackRightYPosInches = -12;


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final Swerve DriveTrain = new Swerve(DrivetrainConstants, new SwerveModuleConstants[] {FrontLeft,
            FrontRight, BackLeft, BackRight});



}
