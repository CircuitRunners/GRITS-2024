package frc.robot.subsystems;
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

/**
 * The ArmSubsystem class controls the arm mechanism of the robot.
 * It provides methods to move, stop, and control the arm's position.
 */
public class arm extends SubsystemBase
{

    /**
     * Creates a new Arm instance.
     * It also sets the neutral mode of the motor to Brake, meaning the motor will stop when no power is applied.
     */
    public class arm 
    {
        private final TalonFX arm;
      /** Creates a new Arm. */

        public void Arm() 
        {
            arm = new TalonFX(ArmConstants.armLeaderId);
            arm.configFactoryDefault(); // Call the method to reset configurations to default
            arm.setNeutralMode(NeutralModeValue.Brake); // Set the neutral mode to Brake
        }
        
        /**
         * Resets the arm encoder to zero.
         * It is helpful when recalibrating the arm's starting position.
         */
        public void resetArmEncoder()
         {
            armLeader.setSelectedSensorPosition(0); // Reset the encoder position to 0
        }
        
    }

    /**
     * Creates a command that stops arm motor
     * Sets motor output to 0, stopping the arm
     * 
     * @return a command that stops the arm when executed
     */
    public CommandBase stopArmCommand()
    {
        return new InstantCommand(() -> arm.set(ControlMode.PercentOutput, 0));
    }

    /**
     * Rotates the arm at a certain rate
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

    //Tarun's methods

    /**
     * Stops the arm motor by setting its output to zero.
     * This method calls the .set() method on the arm motor controller
     * and sets the control mode to PercentOutput with 0 as the output,
     * effectively stopping the motor from moving.
     */
    public void stopArm() 
    {
        arm.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Periodically updates the SmartDashboard with the current arm encoder position.
     * This method calls SmartDashboard.putNumber() to display the arm's current encoder position.
     */
    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("clawEncoderPos", arm.getSelectedSensorPosition());
    }
}
