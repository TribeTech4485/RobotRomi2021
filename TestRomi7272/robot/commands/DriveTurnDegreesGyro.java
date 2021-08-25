// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveTurnDegreesGyro extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final RomiDrivetrain drivetrain;

  double degrees;
  double speed;

  private int counter1 = 4;
  //private int counter2 = 19;

  /**
   * Turns a specific number of degrees based on gyro readings, stopping once the angle turned is greater/equal than desired.
   * This command has a tendency to overshoot the desired angle if turning fast or if gyro latency is high.
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param degrees number of degrees to turn, use positive to turn right, negative to turn left.
   * @param speed the speed to drive, use positive only (negative will change direction of angle turn).
   */
  public DriveTurnDegreesGyro(RomiDrivetrain drivetrain, double degrees, double speed) {
    this.drivetrain = drivetrain;
    this.degrees = degrees;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));

    drivetrain.resetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(0, degrees > 0 ? speed : speed * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}. Current Degrees {1}", this.getName(), drivetrain.getAngle()));
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(MessageFormat.format("Current Angle {0} degrees", drivetrain.getAngle()));
    if (counter1++ % 5 == 0) {
      System.out.println("**current angle: "+String.format("%.4f", drivetrain.getAngle())+" target: "+String.format("%.4f", degrees));
    }
    //if (counter2++ % 20 == 0) {
      //drivetrain.printGyroOffsets();
    //}
    return Math.abs(drivetrain.getAngle()) >= Math.abs(degrees);
  }
}
