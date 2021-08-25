// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForwardInches;
import frc.robot.commands.DriveTurnDegrees;
import frc.robot.subsystems.RomiDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveASquare extends SequentialCommandGroup {

  /** Creates a new SquareDriveSequence. */
  public DriveASquare(RomiDrivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    for (int i = 0; i < 4; i++) {
      addCommands(new DriveForwardInches(drivetrain,6,.75));
      addCommands(new DriveTurnDegrees(drivetrain,90,.75));
    }
  }
}
