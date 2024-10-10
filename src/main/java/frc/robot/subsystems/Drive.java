// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Volts;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib.swerve.Swerve;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.FieldUtil;
import frc.lib.utils.PathPlannerUtil;
//import frc.robot.Vision;
import frc.robot.Constants.SwerveConstants;
import frc.robot.io.DriverControls;

public class Drive extends SubsystemBase {
  public static double limit = 1;
  private Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();
  private boolean sysIdTranslator = true;
  private final SysIdSwerveTranslation translation = new SysIdSwerveTranslation();
  private final SysIdRoutine sysIdTranslation = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, 
      Volts.of(7),
      null,
      null),  
    new SysIdRoutine.Mechanism(
      (volts) -> swerve.setControl(translation.withVolts(volts)),
      null,
      this)
    );
  private final SysIdSwerveRotation rotation = new SysIdSwerveRotation();
  private final SysIdRoutine sysIdRotation = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volts.of(7),
      null,
      null),
    new SysIdRoutine.Mechanism(
      (volts) -> swerve.setControl(rotation.withVolts(volts)),
      null,
      this));

  private SlewRateLimiter forwardLimiter, strafeLimiter;
  /** Creates a new Drive */
  public Drive(Swerve swerve) {
    SignalLogger.setPath("logs/sysid/drive");
    this.swerve = swerve;

    forwardLimiter = new SlewRateLimiter(5, -10, 0);
    strafeLimiter = new SlewRateLimiter(5, -10, 0);
    swerve.setPigeonOffset();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Pose2d targetPose = PathPlannerUtil.getCurrent
  }
}
