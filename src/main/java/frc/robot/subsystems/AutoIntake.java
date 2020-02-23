package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoIntake extends SubsystemBase {
  DigitalInput DownS = new DigitalInput(1);
  DigitalInput UpS = new DigitalInput(0);
  Spark SuckDown = new Spark(1);
  Spark SuckUp = new Spark(2);
  SpeedControllerGroup Canon = new SpeedControllerGroup(SuckDown, SuckUp);
  Spark SparkIntake = new Spark(0);
  Solenoid intakeSolenoid = new Solenoid(0);
  public AutoIntake() {
    SparkIntake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Sensor", IsBallD());
    SmartDashboard.putBoolean("Launcher Sensor", IsBallU());
  }

  public boolean IsBallD()
  {
        return DownS.get();
  }

  public boolean IsBallU()
  {
        return UpS.get();
  }

  public void BallOBall()
  {
    if(IsBallU() && !IsBallD())
    {
      Canon.set(-.7);
    }
    else
    {
      Canon.set(0);
    }
  }

  public void IntakeControl()
  {
    if(IsBallU())
    {
      SparkIntake.set(0);
      intakeSolenoid.set(false);
    }
    else
    {
      SparkIntake.set(.8);
      intakeSolenoid.set(true);
    }
    return;
  }

  public void GoUp()
  {
    intakeSolenoid.set(false);
  }

  public void GoDown()
  {
    intakeSolenoid.set(true);
  }

  public void IntakeIt(){
    SparkIntake.set(.8);
  }

  public void NoInte(){
    SparkIntake.set(0);
  }

  public void Defend(){
    SparkIntake.set(-.8);
  }

  public void SuckIt()
  {
    Canon.set(-.7);
  }

  public void SpitIt()
  {
    Canon.set(.7);
  }

  public void StopIt()
  {
    Canon.set(0);
  }
}