// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.swerve.Swerve;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.generated.TunerConstants;
import frc.robot.io.DriverControls;
import frc.robot.io.OperatorControls;
import frc.robot.subsystems.Drive;
 import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */ 
public class Robot extends TimedRobot {
  private Drive drive;
  // private Shooter shooter;
  private DriverControls driverControls;
  private OperatorControls operatorControls;
  private Command m_autonomousCommand;
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>(); 

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    configureSubsystems();
    drive.resetGyro();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  public void driverStationConnected(){
    configureAutos();
    configureBindings();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //shooter.arm.resetTargetAngleToEncoderAngle();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected().get();
    //shooter.arm.resetTargetAngleToEncoderAngle();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //shooter.arm.resetTargetAngleToEncoderAngle();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}



  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureAutos(){
    //PathPlannerUtil.configure(drive,shooter);
    autoChooser.addOption("Do Nothing", () -> Commands.print("Doing Nothing"));
    autoChooser.setDefaultOption("Taxi Service", () ->   (drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.5, 0, 0)).withTimeout(4)));
    //autoChooser.addOption("Shoot + Nick's Taxi Service", () ->   shooter.shootCommand().andThen(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(1.2, 0, 0)).withTimeout(2.5)));
    PathPlannerUtil.getAutos().forEach(path -> {
      autoChooser.addOption(path, () -> PathPlannerUtil.getAutoCommand(path));
    });
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings(){
      driverControls = new DriverControls(0);
      drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)));
      driverControls.robotRelative()
        .whileTrue(Commands.runOnce( () ->drive.setDefaultCommand(drive.driveRobotCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)))))
        .whileFalse(Commands.runOnce(() -> drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)))));
      driverControls.y().onTrue(Commands.runOnce(() -> Drive.limit = 1.0)).onFalse(Commands.runOnce(() -> Drive.limit = 0.8));
      driverControls.a().whileTrue(drive.driveFieldCentricCommand(() -> new ChassisSpeeds(0.5, 0, 0)));
      driverControls.start().onTrue(drive.resetGyroCommand());
    // operatorControls = new OperatorControls(0);
    // operatorControls.start().onTrue(Commands.runOnce(() -> shooter.arm.resetTargetAngleToEncoderAngle()));
    // operatorControls.setArmShootPos().onTrue(shooter.arm.setArmShootPosition());
    // operatorControls.setArmIntakePos().onTrue(shooter.arm.setArmIntakePosition());
    // operatorControls.x().onTrue(shooter.arm.setArmAmpPosition());
    // shooter.arm.runManual(operatorControls.armManual());
    // operatorControls.runFlyWheelOut().whileTrue(shooter.flywheel.setShootSpeedCommand()).onFalse(shooter.flywheel.stopFlywheelCommand());
    // operatorControls.autoIntakeFromSource().whileTrue(shooter.rollers.autoIntake()).onFalse(shooter.rollers.runRollersOutCommandSlow().withTimeout(0.2).finallyDo(() -> shooter.rollers.stopRollers()));
    // operatorControls.runRollersOut().whileTrue(shooter.rollers.runRollersOutCommand()).onFalse(shooter.rollers.stopRollersCommand());
    // operatorControls.shoot().onTrue(shooter.shootCommand());
    // operatorControls.rightTrigger().onTrue(shooter.shootCommand());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void configureSubsystems() {
    drive = new Drive(TunerConstants.DriveTrain);
    // elevator = new Elevator();
    // intake = new Intake();
    // shooter = new Shooter();
  }

}

