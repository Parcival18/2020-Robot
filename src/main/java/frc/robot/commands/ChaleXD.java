
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.AutoIntake;;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class ChaleXD extends SequentialCommandGroup {
  public ChaleXD(Launcher launcher, AutoIntake autoIntake, DriveTrain drive, LimeLight limey) {
    addCommands(

        new InstantCommand(()-> limey.setPipeline(1)),

        new RunCommand(drive::LimeLightAim, drive)
          .withTimeout(2),

        new InstantCommand(launcher::AMidShoot, launcher),
          new WaitCommand(2),

        new RunCommand(autoIntake::SuckIt, autoIntake)
          .withTimeout(3),
        
        new InstantCommand(autoIntake::GoDown, autoIntake),

        new RunCommand(() -> drive.teleopDrive(0, -.5), drive)
          .withTimeout(2),

        new RunCommand(()-> drive.teleopDrive(.7, -.5), drive)
          .withTimeout(2),
        
        new RunCommand(()-> drive.teleopDrive(0, -.5), drive)
          .withTimeout(2),

        new InstantCommand(autoIntake::IntakeIt, autoIntake),

        new RunCommand(()-> drive.teleopDrive(.7, 0), drive)
    );
  }
}