// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/** An example command that uses an example subsystem. */
public class DriveForwardInchesGyroStraightPID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RomiDrivetrain drivetrain;
  private double inches;
  private double speed;

  private PIDController pid;

  private int counter1 = 2;
  private boolean printFlag = false;


  /**
   * Combines the most sensors in an attempt to do completely straight driving with course correction (if bumped) to ensure robot always drives straight.
   * This command uses encoders to measure the inches to drive, as well as a PID-enable gyro to make sure the robot heading stays at angle 0.
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param inches number of inches to drive
   * @param speed the speed (-1 to 1) to drive, use negative to drive backwards.
   * 
   */
  public DriveForwardInchesGyroStraightPID(RomiDrivetrain drivetrain, double inches, double speed) {

    this.drivetrain = drivetrain;
    this.inches = inches;
    this.speed = speed;

    this.pid = new PIDController(0.050, 0, 0);
    this.pid.setTolerance(1, 10);        //within 1 degree of target with 10 data samples

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0} speed: {1} distance: {2}", this.getName(), String.valueOf(speed), String.valueOf(inches)));
    drivetrain.resetEncoders();
    drivetrain.resetAngle();
    System.out.println("**gyro angle after reset in initialize: " + drivetrain.getAngle());
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double reduceFactor;
    double leftSpeed;
    double rightSpeed;

    double currentHeading = drivetrain.getAngle();      // get current heading
    double pidValue = MathUtil.clamp(pid.calculate(currentHeading, 0), -0.3, 0.3);  // check if veered off 0 heading (straight ahead)
    if (pidValue > 0) {
      reduceFactor = 1 - pidValue;   // get factor to reduce speed
    } else {
      reduceFactor = 1 + pidValue;
    }

    if (counter1++ % 2 == 0) {    // only if want to print results to console
      printFlag = true;
      //System.out.println("**pid atSetpoint(): " + pid.atSetpoint());    // check if at setpoint
    } else {
      printFlag = false;
    }
    printFlag = true;   //*** testing  remove !! */

    if (currentHeading >= -0.020 && currentHeading <= 0.020) {
      leftSpeed = speed;    // if heading almost straight ahead no adjustment in speed
      rightSpeed = speed;
      if (printFlag) { System.out.println("**no turn correction go straight ahead: L/R:  "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)+" pid: "+String.format("%.3f", pidValue)+" factor: "+String.format("%.3f", reduceFactor)+" heading:"+String.format("%.3f",currentHeading)); }

    } else if ( currentHeading > 0 ) {    // if heading is pos need to turn to left (slow left side)
      //reduceFactor = 1 + pidValue;      // get factor to reduce speed
      leftSpeed = speed * reduceFactor;   // slow left speed by reduce factor
      rightSpeed = speed;
      if (printFlag) { System.out.println("**turn Left Correction: L/R:  "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)+" pid: "+String.format("%.3f", pidValue)+" factor: "+String.format("%.3f", reduceFactor)+" heading:"+String.format("%.3f",currentHeading)); }

    } else {   // else heading is neg so need to turn to right (slow right side)	
      //reduceFactor = 1 - pidValue;   // get factor to reduce speed   
      leftSpeed = speed;
      rightSpeed = speed * reduceFactor;   // slow right speed by reduce factor
      leftSpeed += 0.05;  // test for when drifting left to increase left side since less powerful
      if (printFlag) { System.out.println("**turn Right Correction: L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)+" pid: "+String.format("%.3f", pidValue)+" factor: "+String.format("%.3f", reduceFactor)+" heading:"+String.format("%.3f",currentHeading)); }

    }
    //if (currentHeading < 0) {
    //  leftSpeed += 0.02;  // test for when drifting left to increase left side since less powerful
    //}

    drivetrain.tankDrive(leftSpeed, rightSpeed);
   
    //Debug the speed calculation
    //if (printFlag) { System.out.println(MessageFormat.format("**GyroPID final speeds: L/R:  {0} ~ {1} pid: {2} heading: {3}", String.format("%.3f",leftSpeed), String.format("%.3f", rightSpeed), String.format("%.3f",pidValue), String.format("%.3f",currentHeading))); }

    /*
    if (pidValue >= -0.015 && pidValue <= 0.015) {
      leftSpeed = speed;  // if almost straight ahead (heading +- 0.5 of straight head) no adjustment in speed
      rightSpeed = speed;
      if (printFlag) { System.out.println("**correct  go straight: L/R:  "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)+" pid: "+String.format("%.3f", pidValue)+" factor: "+String.format("%.3f", reduceFactor)); }
    } else if ( pidValue < 0 ) {  // if pid is neg need to turn to left (slow left side)
      reduceFactor = 1 + pidValue;   // get factor to reduce speed
      leftSpeed = speed * reduceFactor;   // slow left by reduce factor
      rightSpeed = speed;
      if (printFlag) { System.out.println("**turn Left Correction: L/R:  "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)+" pid: "+String.format("%.3f", pidValue)+" factor: "+String.format("%.3f", reduceFactor)); }
    } else {   // else pid is pos so need to turn to right (slow right side)	
      reduceFactor = 1 - pidValue;   // get factor to reduce speed   
      leftSpeed = speed;
      rightSpeed = speed * reduceFactor;   // slow right by pid value or set amount
      if (printFlag) { System.out.println("**turn Right Correction: L/R: "+String.format("%.3f", leftSpeed)+" ~ "+String.format("%.3f", rightSpeed)+" pid: "+String.format("%.3f", pidValue)+" factor: "+String.format("%.3f", reduceFactor)); }
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}", this.getName()));
    drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Finish when the driving distance is met and the robot is at the correct heading.
    //return isDistanceMet() && pid.atSetpoint();
    return isDistanceMet();
  }

  private boolean isDistanceMet() {
    double aveDistance = drivetrain.getAveDistanceInch();
    if (printFlag) { System.out.println(MessageFormat.format("**ave distance: {0}  target: {1}", aveDistance, inches)); }
    return Math.abs(aveDistance) > inches;
  }
}
