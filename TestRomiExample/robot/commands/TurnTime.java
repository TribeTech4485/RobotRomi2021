// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/*
 * Creates a new TurnTime command. This command will turn your robot for a
 * desired rotational speed and time.
 */
public class TurnTime extends CommandBase {

  private final double m_duration;
  private final double m_rotationalSpeed;
  private final DriveTrain m_driveTrain;

  private double startTime = 0.0;
  private int counter = 5;

  /**
   * Creates a new TurnTime.
   *
   * @param speed The speed which the robot will turn. Negative is in reverse.
   * @param time How much time to turn in seconds
   * @param driveTrain The drive subsystem on which this command will run
   */
  public TurnTime(double speedIn, double timeIn, DriveTrain driveTrain) {
    this.m_rotationalSpeed = speedIn;
    this.m_duration = timeIn;
    this.m_driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.doArcadeDrive(0, 0);       // make sure robot is stopped
    startTime = Timer.getFPGATimestamp();   // get start time
    System.out.println(MessageFormat.format("**Started {0}  start time: {1}", this.getName(), String.format("%.3f", startTime)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.doArcadeDrive(0, m_rotationalSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.doArcadeDrive(0, 0);     // make sure robot is stopped
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;    // get elapsed time
    if (counter++ % 5 == 0) { System.out.println("**TurnTime  elapsed: "+String.format("%.3f", elapsedTime)+" duration: "+m_duration); }
    return (elapsedTime >= m_duration);   // check if time to end
  }
}
