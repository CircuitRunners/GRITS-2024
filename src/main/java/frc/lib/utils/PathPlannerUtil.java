package frc.lib.utils;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;


public class PathPlannerUtil {
    private static final DoubleArraySubscriber kTargetPoseSub = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("/Pathplanner/targetPose")
        .subscribe(new double[] {0,0,0});

    public static void configure(Drive drive, Shooter shooter){
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            SwerveConstants.translationalPID, 
            SwerveConstants.rotationalPID, 
            TunerConstants.kSpeedAt12VoltsMps, 
            SwerveConstants.driveBaseRadiusMeter, 
            new ReplanningConfig(true, true)
        );
        
        AutoBuilder.configureHolonomic(
            drive::getPose();
        );

    }
}
