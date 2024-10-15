package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControls extends CommandXboxController {
    
    public OperatorControls(int port){
        super(port);
    }

    public Trigger setArmIntakePos(){
        return povDown();
    }

    public Trigger setArmShootPos(){
        return povUp();
    }

    public double armManual(){
        return MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband);
    }

    public Trigger runFlyWheelOut(){
        return a();
    }

    public Trigger runShooterIn(){
        return b();
    }

    public Trigger runRollersOut(){
        return leftBumper();
      }
    
      public Trigger shoot(){
        return leftTrigger();
      }
      public Trigger autoIntakeFromSource(){
        return rightBumper();
      }

      
      public Trigger armManualUp(){
        return povUp();
      }
    
      public Trigger armManualDown(){
        return povDown();
      }
    
      public Trigger setArmHigh(){
        return new Trigger(() -> MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband) > 0);
      }
      
      public Trigger setArmLow(){
        return new Trigger(() -> MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband) < 0);
      }
    
      public Trigger armDynamicForward() {
        return povUp();
      }
    
      public Trigger armDynamicReverse() {
        return povDown();
      }
    
      public Trigger armQuasistaticForward() {
        return povLeft();
      }
    
      public Trigger armQuasistaticReverse() {
        return povRight();
      }
}
