// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForwardTimed extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RomiDrivetrain drivetrain;

  Timer timer;

  double seconds;
  double speed;

  /**
   * Example driving command for autonomous mode to drive straight without any sensors.
   * Utilizes the WPILib Timer class to measure driving time.
   * Using this command you will likely notice the robot drifting to an angle and not driving straight.
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param seconds number of seconds to drive
   * @param speed the speed (-1 to 1) to drive, use negative to drive backwards.
   */
  public DriveForwardTimed(RomiDrivetrain drivetrain, double seconds, double speed) {
    this.drivetrain = drivetrain;
    this.seconds = seconds;
    this.speed = speed;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Start and reset the timer. If you don't reset the timer, it will contain the last command run's time (and won't do anything).
    timer.start();
    timer.reset();
    System.out.println(MessageFormat.format("Started {0}", this.getName()));
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
    //returns true if the current time is greater than or equal to the seconds passed in.
    return timer.hasElapsed(seconds);
  }
}
