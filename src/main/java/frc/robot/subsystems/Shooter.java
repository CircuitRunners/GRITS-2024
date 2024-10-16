package frc.robot.subsystems;

import frc.robot.subsystems.shooter.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Shooter {
    public Arm arm;
    public Flywheel flywheel;
    public Rollers rollers;

    public Shooter(){
        arm = new Arm();
        flywheel = new Flywheel();
        rollers = new Rollers();
    }

    public Command shootCommand(){
        return Commands.sequence(
            flywheel.setShootSpeedCommand(),
            arm.setArmShootPositionAndWait().withTimeout(4),
            rollers.setRollersSpeedInCommand(),
            Commands.waitSeconds(3),
            flywheel.stopFlywheelCommand(),
            rollers.stopRollersCommand()
        );
    }
}
