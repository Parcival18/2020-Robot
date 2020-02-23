package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BrakeClimb extends SubsystemBase {
  /**
   * Creates a new LiftBrakeSubsystem.
   */

  Solenoid liftBrake = new Solenoid(1);
  
  public BrakeClimb() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Smart Dashboard Items
    SmartDashboard.putBoolean("Lift Brake On", isLiftBrakeOn());
  }
  
  public void liftBrakeOn(){
    liftBrake.set(false);
  }

  public void liftBrakeOff(){
    liftBrake.set(true);
  }


  public boolean isLiftBrakeOn(){
    return liftBrake.get();
  }
}