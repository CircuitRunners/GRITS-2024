package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class arm extends SubsystemBase
{
    

    //Make java doc comments above the methods entailing what they do

    // Karti's methods

    //Tarun's methods

    // Goodnews's methods
    /**
     * Creates a command that stops arm motor
     * Sets motor output to 0, stopping the arm
     * 
     * @return a command that stops the arm when executed
     */
    public CommandBase stopArmCommand ()
    {
        return new InstantCommand(() -> arm.set(ControlMode.PercentOutput, 0));
    }

    /**
     * Rotates the arm at a certain rate
     * Rate is between -1.0(full speed reverse) and 1.0 (full speed forward)
     * scaled by rotation modifier
     * 
     * @param rate the speed at which to rotate the arm
     */
    public void rotateArm(double rate)
    {
        targetSpeed = rate;
        arm.set(ControlMode.PercentOutput, (ArmConstants.rotationModifer * rate));
    }

    /**
     * Creates command to rotate the arm at a specified rate
     * 
     * @param rate the speed at which to rotate the arm
     * @return a command that rotates the arm at the given rate when executed
     */

    public CommandBase rotateArmCommand (double rate)
    {
        return this.runOnce(() -> rotateArm(rate));
    }
}
