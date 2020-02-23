package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    /**
     * Creates a new LauncherSubsystem.
     */

    // Set up motor controllers
    CANSparkMax launcherLeader = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax launcherFollower = new CANSparkMax(2, MotorType.kBrushless);

    // Sets up encoders
    CANEncoder launcherEncoder;

    // Sets up PID Controller
    private CANPIDController pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;

    private double velocitySetpoint = 0.0;

    public Launcher() {
        //Resets motor controllers to default conditions
        launcherLeader.restoreFactoryDefaults();
        launcherFollower.restoreFactoryDefaults();

        launcherLeader.setSmartCurrentLimit(60);//current limit
        launcherFollower.setSmartCurrentLimit(60);

        launcherLeader.setInverted(false);

        //sets up follower and makes the direction inverted to the leader
        launcherFollower.follow(launcherLeader, true);

        //Sets up endcoders
        launcherEncoder = launcherLeader.getEncoder();
        //if encoder is counting down when going forward ajust this setting
        //launcherEncoder.setInverted(false);

        pidController = launcherLeader.getPIDController();

        // PID coefficients these will need to be tuned
        kP = 0.0002; //5e-5; 
        kI =  0;//1e-6;
        kD = 0.0001; 
        kIz = 0; 
        kFF = 0.00017; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
        allowableError = 100; //Lets the system known when the velocity is close enough to launch

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        //SmartDashboard.putNumber("Velocity Set", 0.0);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //Smart Dashboard Items
    SmartDashboard.putNumber("Launcher Velocity", getLauncherVelocity());
    SmartDashboard.putBoolean("At Set Velocity", isAtVelocity());
    SmartDashboard.putNumber("Launcher Setpoint", getVelocitySetpoint());
    //setVelocitySetpoint(SmartDashboard.getNumber("Velocity Set", 0.0));//Make this a slider to change velocity setpoint
  }

  public void manualLanuch(double speed){
    launcherLeader.set(speed);
  }

  public void velocityClosedLoopLaunch(){
    final double p = SmartDashboard.getNumber("P Gain", 0);
    final double i = SmartDashboard.getNumber("I Gain", 0);
    final double d = SmartDashboard.getNumber("D Gain", 0);
    final double iz = SmartDashboard.getNumber("I Zone", 0);
    final double ff = SmartDashboard.getNumber("Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Max Output", 0);
    final double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
        pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; }
    pidController.setReference(velocitySetpoint, ControlType.kVelocity);
  }

  public void setVelocitySetpoint(double velocity){
    velocitySetpoint = velocity;
  }

  public double getVelocitySetpoint(){
    return velocitySetpoint;
  }


  public double getLauncherVelocity(){
    return launcherEncoder.getVelocity();
  }

  public boolean isAtVelocity(){
    double error = getLauncherVelocity() - velocitySetpoint;
    return (Math.abs(error) < allowableError);
  }

  public void StopShoot(){
      velocitySetpoint = 0;
      velocityClosedLoopLaunch();
  }

  public void LongShoot(){
      velocitySetpoint = 4800;
      velocityClosedLoopLaunch();
  }

  public void MidShoot(){
      velocitySetpoint = 3650;
      velocityClosedLoopLaunch();
  }

  public void AMidShoot(){
    velocitySetpoint = 3455;
    velocityClosedLoopLaunch();
}

  public void ShortShoot(){
      velocitySetpoint = 2400;
      velocityClosedLoopLaunch();
  }
}