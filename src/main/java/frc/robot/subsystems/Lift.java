package frc.robot.subsystems;

/* import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced; */
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  /**
   * Creates a new LiftSubsystem.
   * 
   */

  //Set up motor controllers
  WPI_TalonSRX liftFollower = new WPI_TalonSRX(21);
  CANSparkMax lift = new CANSparkMax(9, MotorType.kBrushless);

  //Encoder needs to be pugged into liftLeader and pidgeon into liftFollower
  PigeonIMU pidgey = new PigeonIMU(liftFollower);
 
  public Lift() {

    liftFollower.configFactoryDefault();
    lift.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LiftAmp", lift.getOutputCurrent());
    
  }

  public void manualLift(double move){
    lift.set(move);
  }

  public double getAngle(){
    double[] ypr_deg = new double[3];
    pidgey.getYawPitchRoll(ypr_deg);
		return ypr_deg[0];
  }

  public void resetGyro(){
    pidgey.setYaw(0.0);
  }  
}