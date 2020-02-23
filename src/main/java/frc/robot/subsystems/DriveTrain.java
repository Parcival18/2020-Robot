package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private double kP = 0.02;
  private double kF = 0.15;
    /**
     * Creates a new DriveSubsystem.
     * 
     * 
     */

    // Set up motor controllers
    CANSparkMax leftLeader = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax leftFollower = new CANSparkMax(8, MotorType.kBrushless);
    CANSparkMax rightLeader = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(6, MotorType.kBrushless);

    // Sets up encoders
    CANEncoder leftEncoder;
    CANEncoder rightEncoder;

    // Sets up differental drive
    DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

    // Sets up a global lift subsystem so the pidgeon that is on the liftFollow
    // talon can be used
    Lift drive_LiftSubsystem;
    LimeLight drive_LimeLight;

    // Encoder Ticks to Meter Conversion factor (Calculates the Circumferance of a
    // 6" wheel converts to m and then divides by the number of ticks per rev)
    private double ticksToMeter = (Math.PI * 6 * 0.0254) / 540;

    // Encoder Velocity RPM to Wheel Surface Speed meter/second conversion factor
    private double rpmToMeterPerSec = (Math.PI * 6 * 0.0254) / 60;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    public DriveTrain(Lift lift, LimeLight limey) {
    //the liftSubsytem passed into the construstor is set into the global lift subsystem
    drive_LiftSubsystem = lift;
    drive_LimeLight = limey;

    //Resets motor controllers to default conditions
    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.setOpenLoopRampRate(.7);
    rightLeader.setOpenLoopRampRate(.7);

    leftLeader.setSmartCurrentLimit(60);
    leftFollower.setSmartCurrentLimit(60);
    rightLeader.setSmartCurrentLimit(60);
    rightFollower.setSmartCurrentLimit(60);
  

    //Sets up follower motors
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    //Sets up endcoders
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPositionMeter(),
    getRightEncoderPositionMeter());


    //Smart Dashboard Items
    SmartDashboard.putNumber("Left Drive Encoder Position", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Drive Encoder Position", getRightEncoderPosition());
    //SmartDashboard.putNumber("Drive Angle", getAngle());
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocityMeterPerSec(), getRightEncoderVelocityMeterPerSec());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }



 

  public void teleopDrive(double move, double turn) {
    drive.arcadeDrive(move, turn);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);

  }


  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }



  public double getLeftEncoderPosition(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoderPosition(){
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPositionMeter(){
    return leftEncoder.getPosition() * ticksToMeter;
  }

  public double getRightEncoderPositionMeter(){
    return rightEncoder.getPosition() * ticksToMeter;
  } 
  
    /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderPositionMeter() {
    return (getLeftEncoderPositionMeter() + getRightEncoderPositionMeter()) / 2.0;
  }

  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity(){
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocityMeterPerSec(){
    return getLeftEncoderVelocity() * rpmToMeterPerSec;
  }

  public double getRightEncoderVelocityMeterPerSec(){
    return getRightEncoderVelocity() * rpmToMeterPerSec;
  }

  ///Need to add encoder rate meter/second

  //Returns heading 180 to -180.  Right turn is negative and Left turn is positive
  public double getHeading(){
    return Math.IEEEremainder(drive_LiftSubsystem.getAngle(), 360);
  }

  public void zeroHeading(){
    drive_LiftSubsystem.resetGyro();
  }

  public void LimeLightAim()
  {
    double error = drive_LimeLight.getHorizontalOffset();
    kF = Math.copySign(kF, error);
    double outF = kF;             
    double outP = kP * error;
    double outputTurn = outF + outP;
    teleopDrive(0, outputTurn);
  }

  
}