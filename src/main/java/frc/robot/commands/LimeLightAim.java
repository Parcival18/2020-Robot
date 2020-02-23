/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimeLight;

public class LimeLightAim extends CommandBase {
  /**
   * Creates a new LimeLightAim.
   */
  Lift LLlift = new Lift();
  LimeLight LLlimey = new LimeLight();
  DriveTrain LLDrive = new DriveTrain(LLlift, LLlimey);

  private double kP = 0.03;
  //private double kD = 0.0;
  //private double kI = 0.0;
  private double kF = 0.15;

  //private double lastTimestamp = Timer.getFPGATimestamp();
  //private double errorSum = 0;
  //private double lastError = 0;
  //private double iZone =  10.0;
  //StringBuilder outputString = new StringBuilder();

  public LimeLightAim(Lift lift, DriveTrain dTrain, LimeLight limeLight ) {
    // Use addRequirements() here to declare subsystem dependencies.
    LLlift = lift;
    LLDrive = dTrain;
    LLlimey = limeLight; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = LLlimey.getHorizontalOffset();
    //outputString.append("error: " + String.format("%.3f", error));
    //double dt = Timer.getFPGATimestamp() - lastTimestamp;
    //outputString.append("dt: " + String.format("%.3f", dt));
    kF = Math.copySign(kF, error);
    SmartDashboard.putNumber("kF", kF);
    SmartDashboard.putNumber("Error", error);

    /* if(Math.abs(error) < iZone){
      errorSum = errorSum + error * dt;
    }else{
      errorSum = 0;
    } */
    //String strDouble = String.format("%.2f", 2.00023); 

    //double errorRate = (error - lastError)/dt;
    double outF = kF;             //Feed forward output
    //outputString.append("Feed Forward: " + String.format("%.3f", outF));
    double outP = kP * error;     //Proportional output
    //outputString.append("    Proportional: " + String.format("%.3f",outP));
    //double outI = kI * errorSum;  //Intigrator output
    //outputString.append(" ersum: " + String.format("%.3f",errorSum));
    //outputString.append("    Intigrator: " + String.format("%.3f",outI));
    //double outD = kD * errorRate; //Derivitive output
    //outputString.append("    Derivitive: " + String.format("%.3f",outD));


    double outputTurn = outF + outP; //+ outI + outD;


    //outputString.append("    Total Out: " + String.format("%.3f",outputTurn));
    //System.out.println(outputString.toString());
    //outputString.setLength(0);
    LLDrive.teleopDrive(0, outputTurn);

    //SmartDashboard.putNumber("Vision Turn Value", outputTurn);

    //lastTimestamp = Timer.getFPGATimestamp();
    //lastError = error;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LLDrive.teleopDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
