// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTurnDegreesGyroPID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final RomiDrivetrain drivetrain;

  double degrees;
  double speed;
  double speedContinuous;
  boolean continuous;
  PIDController pid;

  /**
   * Turns a specific number of degrees based on gyro readings, using PID to stop at the angle and correct under/over shooting the target angle
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param degrees number of degrees to turn, use positive to turn right, negative to turn left.
   * @param speed the Proportion to use in the PID calculation, use positive only (negative will change direction of angle turn).
   * @param continuous if true, will not end command on target and continue running PID indefinately.
   */
  public DriveTurnDegreesGyroPID(RomiDrivetrain drivetrain, double degrees, double speed, boolean continuous) {
    // I've seen many teams never set any ID values unless they are needed (never meeting target, too much oscillation).
    this.pid = new PIDController(speed, 0, 0);

    //within 1 degree of target with 10 data samples
    this.pid.setTolerance(1, 10);

    this.drivetrain = drivetrain;
    this.degrees = degrees;
    this.speed = speed;
    this.speedContinuous = speed;
    this.continuous = continuous;
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
    double curr_angle = drivetrain.getAngle();
    double pid_calc = pid.calculate(curr_angle, degrees)/100;
 
    double speed;
    if(continuous) { 
      speed = speedContinuous;   // check if continuous
    } else if(pid_calc < 0) {
      speed = Math.min(pid_calc, -.60);
  
    } else {
      speed = Math.max(pid_calc, .60);
    }

    drivetrain.arcadeDrive(0, speed);
    System.out.println(MessageFormat.format("**current gyro angle: {0} \t speed: {1}", curr_angle, speed));
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
    if(continuous) {
      return false;
    }

    if(pid.atSetpoint()) {
      System.out.println(MessageFormat.format("Ending {0}, current angle: {1}", this.getName(), drivetrain.getAngle()));
      return true;
    }
    return false;
  }
}
