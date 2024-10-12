// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldConstants;
import frc.robot.io.DriverControls;
import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class AimAtSpeaker extends Command {
  private Drive drive;
  private Translation2d difference;
  private DriverControls controls;
  private double targetAngle;
  private boolean rotateAroundPose;


  
  //Creates a new ExampleCommand
  public AimAtSpeaker(Drive swerve,DriverControls controls, boolean rotateAroundPose) {
    this.drive = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.controls = controls;
    this.rotateAroundPose = rotateAroundPose; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
      difference = FieldConstants.SpeakerK.kBlueCenterOpening.toTranslation2d().minus(this.drive.getPose().getTranslation());
      targetAngle = Math.atan2(difference.getY(), difference.getX());
    } else {
      difference = FieldConstants.SpeakerK.kRedCenterOpening.toTranslation2d().minus(this.drive.getPose().getTranslation());
      targetAngle = Math.atan2(difference.getY(), difference.getX());
    }

    if(rotateAroundPose) drive.targetAngleDrive(difference,controls);
    else drive.targetAngleDrive(Rotation2d.fromRadians(targetAngle), controls);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
