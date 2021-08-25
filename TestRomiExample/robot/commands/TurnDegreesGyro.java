// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class TurnDegreesGyro extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double m_turnSpeed;
  private double m_degrees;
  private double degreesAbs;

  private double currentHeading;
  private double currentDiff;

  private int counter1 = 5;
  private int counter2 = 5;

  /**
   * Turns a specific number of degrees based on gyro readings, stopping once the angle turned is greater/equal than desired.
   * This command has a tendency to overshoot the desired angle if turning fast or if gyro latency is high.
   *
   * @param speed the speed to turn, use positive only (negative will change direction of angle turn).
   * @param degrees number of degrees to turn, use positive to turn right, negative to turn left.
   * @param driveTrain instance of RomiDrivetrain
   */
  public TurnDegreesGyro(double speedIn, double degreesIn, DriveTrain driveTrain) {

    this.m_driveTrain = driveTrain;
    this.m_turnSpeed = speedIn;
    this.m_degrees = degreesIn;
    this.degreesAbs = Math.abs(degreesIn);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));
    m_driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation;
    currentHeading = m_driveTrain.getGyroAngleZ();
    currentDiff = degreesAbs - Math.abs(currentHeading);  // check current robot heading vs target degrees

    if (currentDiff >= -DriveConstants.kToleranceDegrees && currentDiff <= DriveConstants.kToleranceDegrees) {
      rotation = 0;
      currentHeading = degreesAbs;    // if within tolerance set heading so will finish and end command
      System.out.println("**Turn Gyro done - diff: "+String.format("%.3f", currentDiff)+" heading: "+String.format("%.3f", currentHeading)+" target: "+m_degrees); 
    } else {
      if (m_degrees >= 0) {
        rotation = m_turnSpeed;
      } else {
        rotation = m_turnSpeed * -1;
      }
    }
    m_driveTrain.doArcadeDrive(0, rotation);  // turn using arcadeDrive(xSpeed, zRotation)
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}. Current Degrees {1}", this.getName(), m_driveTrain.getGyroAngleZ()));
    m_driveTrain.doArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter1++ % 5 == 0) {
      System.out.println("**current gyro angleZ: "+String.format("%.4f", currentHeading)+" target: "+String.format("%.4f", m_degrees));
    }
    if (counter2++ % 5 == 0) {
      m_driveTrain.printGyroOffsets();
    }
    return Math.abs(currentHeading) >= degreesAbs;  // within tolerance is checked in execute() method above
  }
}
