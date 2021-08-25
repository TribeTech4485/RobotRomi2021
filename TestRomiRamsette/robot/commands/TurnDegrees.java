// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;

  private int count1 = 4;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, Drivetrain drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. Standard Romi Chassis found here: 
       https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm or 5.551 inches. 
       We then take into consideration the width of the tires.
       ** convert inches to meters since distance methods measure in meters
    */
    //double inchPerDegree = Math.PI * 5.551 / 360;
    double meterPerDegree = Math.PI * (5.551 / 39.37) / 360;
    double targetDist = meterPerDegree * m_degrees;
    double aveTurnDist = getAverageTurningDistance();
    // Compare distance travelled from start to distance based on degree turn
    if (++count1 % 5 == 0) {
      System.out.println("meter/deg: "+String.format("%.3f", meterPerDegree)+" ave dist: "+String.format("%.3f", aveTurnDist)+" target: "+String.format("%.3f", targetDist));
    }
    return getAverageTurningDistance() >= targetDist;
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceMeter());
    double rightDistance = Math.abs(m_drive.getRightDistanceMeter());
    //System.out.println("**turn dist:  left ~ right: " +String.format("%.4f",leftDistance)+" ~ "+String.format("%.4f",rightDistance));
    return (leftDistance + rightDistance) / 2.0;
  }
}
