package frc.lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;
import frc.robot.io.DriverControls;
import frc.robot.subsystems.Drive;

public class SwerveConfig {
  //TODO: tune configs
  private static final Slot0Configs driveConfigs = new Slot0Configs() {
    {
      kP = SwerveConstants.driveKP;
      kI = SwerveConstants.driveKI;
      kD = SwerveConstants.driveKD;
      kS = SwerveConstants.driveKS;
      kV = SwerveConstants.driveKV;
      kG = SwerveConstants.driveKG;
      kA = SwerveConstants.driveKA;
    }
  };
  private static final Slot0Configs turnConfigs = new Slot0Configs() {
    {
      kP = SwerveConstants.angleKP;
      kI = SwerveConstants.angleKI;
      kD = SwerveConstants.angleKD;
      kS = SwerveConstants.angleKS;
      kV = SwerveConstants.angleKV;
      kG = SwerveConstants.angleKG;
      kA = SwerveConstants.angleKA;
    }
  };

  private static final SwerveModuleConstantsFactory CONSTANTS_FACTORY = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(SwerveConstants.driveGearRatio)
      .withSteerMotorGearRatio(SwerveConstants.angleGearRatio)
      .withWheelRadius(Units.metersToInches(SwerveConstants.wheelCircumference / (2 * Math.PI)))
      .withSlipCurrent(SwerveConstants.slipCurrent)
      .withSteerMotorGains(turnConfigs)
      .withDriveMotorGains(driveConfigs)
      .withSpeedAt12VoltsMps(SwerveConstants.maxVelocityMPS)
      .withSteerInertia(SwerveConstants.steerInertia)
      .withDriveInertia(SwerveConstants.driveInertia)
      .withSteerMotorInverted(false)
      .withCouplingGearRatio(SwerveConstants.couplingGearRatio);

  private static final SwerveModuleConstants generateConstants(int turnID, int driveID, int canCoderID,
      Translation2d position, double absoluteOffset) {
    return CONSTANTS_FACTORY.createModuleConstants(turnID, driveID, canCoderID, absoluteOffset, position.getX(),
        position.getY(), false);

  }

  public static Swerve getConfiguredDrivetrain() {
    var drivetrain = new SwerveDrivetrainConstants()
        .withPigeon2Id(SwerveConstants.pigeonID)
        .withCANbusName(SwerveConstants.CANBusName);
    var frontLeft = generateConstants(SwerveConstants.Mod0.angleMotorID, SwerveConstants.Mod0.driveMotorID,
        SwerveConstants.Mod0.canCoderID, SwerveConstants.Mod0.position, SwerveConstants.Mod0.angleOffset.getRadians());
    var frontRight = generateConstants(SwerveConstants.Mod1.angleMotorID, SwerveConstants.Mod1.driveMotorID,
        SwerveConstants.Mod1.canCoderID, SwerveConstants.Mod1.position, SwerveConstants.Mod1.angleOffset.getRadians());
    var backLeft = generateConstants(SwerveConstants.Mod2.angleMotorID, SwerveConstants.Mod2.driveMotorID,
        SwerveConstants.Mod2.canCoderID, SwerveConstants.Mod2.position, SwerveConstants.Mod2.angleOffset.getRadians());
    var backRight = generateConstants(SwerveConstants.Mod3.angleMotorID, SwerveConstants.Mod3.driveMotorID,
        SwerveConstants.Mod3.canCoderID, SwerveConstants.Mod3.position, SwerveConstants.Mod3.angleOffset.getRadians());

    return new Swerve(drivetrain,
        new SwerveModuleConstants[] { frontLeft, frontRight, backLeft, backRight });
  }

  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public static final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public static final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();
  public static final SwerveRequest.ApplyChassisSpeeds applyChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();

  public static ChassisSpeeds toChassisSpeeds(DriverControls driverControls, Drive drive) {
    return new ChassisSpeeds(driverControls.driveForward(), driverControls.driveStrafe(),
            driverControls.driveRotation());
  }
}