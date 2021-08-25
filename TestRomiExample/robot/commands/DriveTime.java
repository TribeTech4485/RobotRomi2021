// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTime extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final double m_duration;
  private final double m_speed;

  private double startTime = 0.0;
  private int counter = 10;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speedIn The speed which the robot will drive. Negative is in reverse.
   * @param timeIn How much time to drive in seconds
   * @param driveTrain The drivetrain subsystem on which this command will run
   */
  public DriveTime(double speedIn, double timeIn, DriveTrain driveTrain) {
    
    this.m_speed = speedIn;
    this.m_duration = timeIn;
    this.m_driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.doTankDrive(0, 0);         // make sure motors are stopped
    startTime = Timer.getFPGATimestamp();   // get start time
    System.out.println(MessageFormat.format("**Started {0}  start time: {1}", this.getName(), String.format("%.3f", startTime)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.doTankDrive(m_speed, m_speed);   // set motors to requested speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();      // make sure motors are stopped
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;    // get elapsed time
    if (counter++ % 10 == 0) { System.out.println("**AutoDriveStraightTime  elapsed: "+String.format("%.3f", elapsedTime)+" duration: "+m_duration); }
    return (elapsedTime >= m_duration);   // check if time to end
  }
}
