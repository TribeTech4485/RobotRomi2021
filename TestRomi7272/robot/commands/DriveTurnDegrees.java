// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTurnDegrees extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  // Calculate the Romi wheel's turning circle circumference to use with the encoder output.
  private static final double ROMI_DIAMETER = 5.75; //Based on measuring the outer edge of the wheels
  private static final double ROMI_CIRCUMFERENCE = Math.PI * ROMI_DIAMETER;

  private final RomiDrivetrain drivetrain;

  double degrees;
  double speed;
  double distanceToTravel;

  private int counter = 4;

  /**
   * Turns a specific number of degrees based on encoder input only (no gyro).
   * Uses the drivetrain turning circumference (diameter of the circle = distance between wheels).
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param degrees number of degrees to turn, use positive to turn right, negative to turn left.
   * @param speed the speed to drive, use positive only (negative will change direction of angle turn).
   */
  public DriveTurnDegrees(RomiDrivetrain drivetrain, double degrees, double speed) {
    this.drivetrain = drivetrain;
    this.degrees = degrees;
    this.speed = speed;

    // Calculate the inches of the circumference to travel based on percent degrees of a circle.
    // For example turning 90deg is ROMI_CIRCUMFERENCE * .25 (1/4th of a circle).
    this.distanceToTravel = Math.abs(ROMI_CIRCUMFERENCE * degrees/360);
    
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
    drivetrain.arcadeDrive(0, degrees < 0 ? speed * -1 : speed);
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
    if (counter++ % 5 == 0) {
      System.out.println("**left dist: "+String.format("%.4f", drivetrain.getLeftDistanceInch())+" right: "+String.format("%.4f", drivetrain.getRightDistanceInch())+" target: "+String.format("%.4f", distanceToTravel));
    }
      return Math.abs(drivetrain.getLeftDistanceInch()) > distanceToTravel && Math.abs(drivetrain.getRightDistanceInch()) > distanceToTravel;
  }
}
