package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.ArmConstants;

/**
 * The ArmSubsystem class controls the arm mechanism of the robot.
 * It provides methods to move, stop, and control the arm's position.
 */
public class Arm extends SubsystemBase
{
        private final TalonFX arm;
        private Rotation2d targetAngle;
        private DutyCycleEncoder throughBore = new DutyCycleEncoder(ArmConstants.throuhBoreEncoderPort);       
        public Arm ()
        {
            arm = new TalonFX(0);
        }
    

    /**
     * Creates a command that stops arm motor
     * Sets motor output to 0, stopping the arm
     * 
     * @return a command that stops the arm when executed
     */
    public Command stopArmCommand()
    {
        return new InstantCommand(() -> stopArm());
    }

    /**
     * Rotates the arm at a certain rate
     * @param rate the speed at which to rotate the arm
     */
    public void rotateArm(double rate)
    {
        arm.set(rate);
    }

    /**
     * Creates command to rotate the arm at a specified rate
     * 
     * @param rate the speed at which to rotate the arm
     * @return a command that rotates the arm at the given rate when executed
     */
    public Command rotateArmCommand (double rate)
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
        arm.set(0);
    }

    /**
     * Periodically updates the SmartDashboard with the current arm encoder position.
     */
    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("clawEncoderPos", arm.getPosition().getValueAsDouble());
    }

    public Command setArmShootPositionAndWait(){
        return run(() -> setTargetAngle(ArmConstants.shootRotation)).until(() -> isArmAtTarget(Units.degreesToRadians(0.5)));
    }

    public void setTargetAngle(Rotation2d targetAngle){ 
        this.targetAngle = Rotation2d.fromRadians(MathUtil.clamp(targetAngle.getRadians(), ArmConstants.minRadians, ArmConstants.maxRadians));
    }

    public boolean isArmAtTarget(double threshold){
        return MathUtil.isNear(targetAngle.getDegrees(), getArmRotation().getDegrees(),threshold);
    }

    public Rotation2d getArmRotation(){
        return Rotation2d.fromRadians(throughBore.getAbsolutePosition() * 2 * Math.PI).minus(Rotation2d.fromDegrees(350));
    }


    public void resetTargetAngleToEncoderAngle() {
        setTargetAngle(getArmRotation());
    }

    public double getRawEncoderValue(){
        return throughBore.getAbsolutePosition();
    }

    public void armVoltage(Measure<Voltage> voltageMeasure){
        arm.setVoltage(voltageMeasure.magnitude());
    }

    public Command setArmAmpPosition(){
        return runOnce(() -> setTargetAngle(ArmConstants.ampRotation));
      }
    
    public Command setArmShootPosition(){
        return runOnce(() -> setTargetAngle(ArmConstants.shootRotation));
    }

    public Command setArmIntakePosition(){
        return runOnce(() -> setTargetAngle(ArmConstants.intakeRotation));
    }

    public void runManual(double value){
        setTargetAngle(targetAngle.plus(Rotation2d.fromDegrees(value * ArmConstants.armSpeed)));
    }
    
}
