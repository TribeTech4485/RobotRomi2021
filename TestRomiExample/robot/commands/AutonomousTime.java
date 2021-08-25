// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousTime extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param driveTrain The drive subsystem on which this command will run
   */
  public AutonomousTime(DriveTrain driveTrain) {
    addCommands(
        new DriveTime(0.75, 2.0, driveTrain),
        new TurnTime(0.65, 1.5, driveTrain),
        new DriveTime(0.75, 2.0, driveTrain),
        new TurnTime(-0.65, 1.5, driveTrain));
  }
}
