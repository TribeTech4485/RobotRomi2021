// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param driveTrain The driveTrain subsystem on which this command will run
   */
  public AutonomousDistance(DriveTrain driveTrain) { 
    addCommands(
        new DriveDistance(0.75, 15, driveTrain),
        new TurnDegreesGyro(0.55, 180, driveTrain),
        new DriveDistance(0.75, 15, driveTrain),
        new TurnDegreesGyro(0.55, 180, driveTrain));
  }
}
