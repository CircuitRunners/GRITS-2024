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
    // Karti's methods
    /**
     * Creates a new Arm instance.
     * This constructor initializes the TalonFX motor controller and sets it to factory default.
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
         * This method is used to reset the encoder position of the arm motor to 0.
         * It is helpful when recalibrating the arm's starting position.
         */
        public void resetArmEncoder()
         {
            armLeader.setSelectedSensorPosition(0); // Reset the encoder position to 0
        }
        
    }

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

    //Tarun's methods

    /**
     * Calls .set method for object arm
     * Sets first param as the control mode percent output and the second as 0
     * Access specifier is public
    */
    public void stopArm() 
    {
        arm.set(ControlMode.PercentOutput,0)
    }

    /**
     * Calls .putNumber for variable SmartDashboard
     * Sets first param to be clawEncoderPos and the second as the arm variable calling .getSelectedSensorPositon
     */
    public void periodic() 
    {
        SmartDashboard.putNumber(clawEncoderPos, arm.getSelectedSensorPositon)
    }
}
