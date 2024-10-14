public class arm {
      /** Creates a new Arm. */
    public void Arm() {
        TalonFX arm = new TalonFX(ArmConstants.armLeaderId);
        arm.configFactoryDefault(); // Call the method to reset configurations to default
        arm.setNeutralMode(NeutralModeValue.Brake); // Set the neutral mode to Brake

        public void resetArmEncoder() {
            armLeader.setSelectedSensorPosition(0); // Reset the encoder position to 0
        }
        
    }
    
}
