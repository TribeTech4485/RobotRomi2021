// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTurnNetworkTableAnglePID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final RomiDrivetrain drivetrain;

  double speed;
  boolean continuous;
  PIDController pid;
  NetworkTableEntry angleEntry;
  NetworkTableEntry objectCountEntry;

  /**
   * Turns an angle (using PID + gyro) by reading correction values stored in a network table entry
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param angleEntry NetworkTableEntry that contains a double value for angle correction.
   * @param objectCountEntry NetworkTableEntry that contains the number of detected objects in the vision program.
   */
  public DriveTurnNetworkTableAnglePID(RomiDrivetrain drivetrain, NetworkTableEntry angleEntry,
      NetworkTableEntry objectCountEntry, double speed, boolean continuous) {
    this.pid = new PIDController(speed, 0, 0);
    
    //within 1 degree of target with 10 data samples
    this.pid.setTolerance(1, 10);

    this.drivetrain = drivetrain;
    this.speed = speed;
    this.continuous = continuous;
    this.angleEntry = angleEntry;
    this.objectCountEntry = objectCountEntry;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int objectCount = objectCountEntry.getNumber(0).intValue();

    // Don't drive or calculate pid if there multiple objects detected.
    // The vision program is likely confused on its target and may need to be calibrated.
    if (objectCount > 1) {
      return;
    }

    double curr_angle = angleEntry.getDouble(0) * -1;
    double pid_calc = pid.calculate(curr_angle, 0) / 100;

    double speed;
    if (pid_calc < 0) {
      speed = Math.min(pid_calc, -.25);
    } else {
      speed = Math.max(pid_calc, .25);
    }

    drivetrain.arcadeDrive(0, speed);
    System.out.println(MessageFormat.format("current network table angle, speed {0}\t{1}", curr_angle, speed));
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
    // continuous=true mode is useful for debugging PID calibration
    // with some work it could also be used for autonomous to make sure the robot is still on target angle (corrected when bumped).
    if (continuous) {
      return false;
    }

    if (pid.atSetpoint()) {
      System.out.println(MessageFormat.format("Ending {0}, current angle: {1}", this.getName(), drivetrain.getAngle()));
      return true;
    }
    return false;
  }
}
