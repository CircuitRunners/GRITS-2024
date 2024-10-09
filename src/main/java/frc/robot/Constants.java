package frc.robot;

//Measurement Units
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

//Pathplanner
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

//AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Geometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;

//Kinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

//Units and stuff
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

//Swerves and utils
import frc.lib.swerve.SwerveModuleConstants;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.utils.AllianceFlipUtil;

public final class Constants{
  public static final class SwerveConstants{
    public static final int pigeonID = 1;
    public static final boolean invertGyro = true; //Always make sure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2); //Must be tuned to specific robot

    /** Drivetrain Constants 
     * Must be tuned to specific robot
    */
    public static final double trackWidth = Units.inchesToMeters(23.736);
    public static final double wheelBase = Units.inchesToMeters(23.736);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /**
     * Swerve kinematics
     * Don't change this unless you're not doing a traditional 4 module rectangle/square swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      Mod0.position,
      Mod1.position,
      Mod2.position,
      Mod3.position);
    
    /** Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /**Motor Inverts*/
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert - chosenModule.driveMotorInvert;

    /** Angle encoder invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /**Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 25;
    public static final int drivePeakCurrentLimit = 40;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /** Angle Motor PID Values */
    public static final double angleKP = TunerConstants.steerGains.kP;
    public static final double angleKI = TunerConstants.steerGains.kI;
    public static final double angleKD = TunerConstants.steerGains.kD;
    public static final double angleKF = 0;

    /**Angle Motor SYSID values */
    public static final double angelKS = TunerConstants.steerGains.kS;
    public static final double angleKV = TunerConstants.steerGains.kV;
    public static final double angleKG = TunerConstants.steerGains.kG;
    public static final double angleKA = TunerConstants.steerGains.kA;

    /**Drive Motor PID Values */
    public static final double driveKP = TunerConstants.driveGains.kS;
    public static final double driveKI = TunerConstants.driveGains.kI;
    public static final double driveKD = TunerConstants.driveGains.kD;
    public static final double angleKF = 0;

    /** Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE*/
    public static final double driveKS = TunerConstants.driveGains.kS;
    public static final double driveKV = TunerConstants.driveGains.kV;
    public static final double driveKG = TunerConstants.driveGains.kG;
    public static final double driveKA = TunerConstants.driveGains.kA;

    /** Swerve Profiling Values
     * Meters per second
    */
    public static final double maxVelocityMPS = TunerConstants.kSpeedAt12VoltsMps;//Must be tuned to specific robot
    public static final double maxModuleVelocityMPS = maxVelocityMPS;
    public static final double maxModuleAccelerationMPSSq = 2.0; //Must be tuned to specific robot

    /**Radians per Second */
    public static final double maxAngularVelocityRPS = 10.0;//Must be tuned to specific robot
    public static final double maxAngularAccelerationRPSSq = 5.0;//Must be tuned to specific robot

    public static final double slipCurrent = 300;
    public static final double steerInertia = TunerConstants.kSteerInertia;
    public static final double driveInertia = TunerConstants.kDriveInertia;
    public static final double couplingGearRatio = TunerConstants.kCoupleRatio;
    public static final String CANBusName = TunerConstants.kCANbusName;
    public static final double driveBaseRadiusMeter = Units.inchesToMeters(12);
    public static final Translation2d[] modulePositions = { Mod0.position, Mod1.position, Mod2.position,
        Mod3.position };

    /**PID Constants */
    public static final PIDConstants translationalPID = new PIDConstants(driveKP, driveKI, driveKD);
    public static final PIDConstants rotationalPID = new PIDConstants(angleKP, angleKI, angleKD);

    /**Front Left Module - Module 0 */
    //Must be tuned to specific robot
    public static final class Mod0{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 0;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kFrontLeftEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter/2.0,wheelBase/2.0);//front left +x,+y
    }

    /**Front Right Module - Module 1 */
    //Must be tuned to specific robot
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kFrontRightEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter / 2.0, -wheelBase / 2.0); //front right is +x, -y
    }
  
    /**Back Left Module - Module 2 */
    //Must be tuned to specific robot
    public static final class Mod2{
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kBackLeftEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter/-2.0, wheelBase/2.0);//Back left -x,+y
    }
  
    /**Back Right Module - Module 3 */
    //Must be tuned to specific robot
    public static final class Mod3{
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;

      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kBackRightEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID);
      public static final Translation2d position = new Translation2d(-driveBaseRadiusMeter/2.0,-wheelBase/2.0);// -x,-y
    }
  
    public static final PathConstraints pathConstraints = new PathConstraints(maxVelocityMPS, maxModuleAccelerationMPSSq, maxAngularVelocityRPS, maxAngularAccelerationRPSSq);
  }

  public static final class DriverConstants{
    public static final double stickDeadband = 0.2;
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
  }

  public static final class ElevatorConstants{
    public static final int lMotor = 0;
    public static final int rMotor = 0;
    public static final double ks = 0;
    public static final double kg = 0;
    public static final double kv = 0;
    public static final double high = 0;
    public static final double mid = 0;
    public static final double low = 0;
    public static final double kP = 0;
    
  }
}
