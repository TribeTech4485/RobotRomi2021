// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTurnDegreesGyroPID;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * A complex autonomous command that spins continuously for a few seconds
 */
public class AutoSpinContinuous extends SequentialCommandGroup {

  /** Creates a new AutoSpinContinuous.
   *
   * @param driveTrain The RomiDrivetrain subsystem this command will run on
   */
  public AutoSpinContinuous(RomiDrivetrain drivetrain) {

    addCommands(
      new InstantCommand(() -> drivetrain.stop(), drivetrain),    // make sure stopped
      new DriveTurnDegreesGyroPID(drivetrain, 90, .75, true).withTimeout(4.5),
      new InstantCommand(drivetrain::stop, drivetrain)       // make sure stopped
    );
  }
}
