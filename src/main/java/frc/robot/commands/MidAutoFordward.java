
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.AutoIntake;;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class MidAutoFordward extends SequentialCommandGroup {
  public MidAutoFordward(Launcher launcher, AutoIntake autoIntake, DriveTrain drive) {
    addCommands(

        new InstantCommand(launcher::AMidShoot, launcher),
          new WaitCommand(3),

        new RunCommand(autoIntake::SuckIt, autoIntake)
          .withTimeout(3),

        new RunCommand(() -> drive.teleopDrive(.5, 0), drive)
          .withTimeout(1)
    );
  }
}