// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForwardInches extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RomiDrivetrain drivetrain;

  double inches;
  double speed;

  /**
   * Drive forward the specified number of inches at the specified speed based on encoder readings
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param inches number of inches to drive
   * @param speed the speed (-1 to 1) to drive, use negative to drive backwards.
   * 
   */
  public DriveForwardInches(RomiDrivetrain drivetrain, double inches, double speed) {
    this.drivetrain = drivetrain;
    this.inches = inches;
    this.speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}", this.getName()));
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Using absolute to measure change of distance with encoders, and not direction.
    // I chose to handle direction with pos/neg speed.
    return Math.abs(drivetrain.getLeftDistanceInch()) > inches && Math.abs(drivetrain.getRightDistanceInch()) > inches;
  }
}
