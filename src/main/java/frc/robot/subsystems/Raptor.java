package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Raptor extends SubsystemBase {

    DoubleSolenoid hangerSolenoid = new DoubleSolenoid(3 , 4);

    private boolean isClimbHookClosed = false;

    DoubleSolenoid Mouth = new DoubleSolenoid(2, 5);

    public Raptor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Smart Dashboard Items
    SmartDashboard.putBoolean("Is Climb Hook Closed", isClimbHookClosed);
  }

  public void closeClimbHook(){
    hangerSolenoid.set(Value.kForward);
    isClimbHookClosed = true;
  }

  public void openClimbHook(){
    hangerSolenoid.set(Value.kReverse);
    isClimbHookClosed = false;
  }

  public void parkClimbHook(){
    hangerSolenoid.set(Value.kOff);
    isClimbHookClosed = false;
  }

  public boolean isClimbHookClosed(){
    return isClimbHookClosed;
  }

  public void OpenMouth(){
      Mouth.set(Value.kForward);
  }

  public void CloseMouth(){
      Mouth.set(Value.kReverse);
  }
}