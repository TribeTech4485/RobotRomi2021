// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesUnits extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final double m_degrees;
  private final double m_speed;

  private int counter = 5;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param driveTrain The driveTrain subsystem on which this command will run
   */
  public TurnDegreesUnits(double speed, double degrees, DriveTrain driveTrain) {
    this.m_degrees = degrees;
    this.m_speed = speed;
    this.m_driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_driveTrain.doArcadeDrive(0, 0);   // Set motors to stop
    m_driveTrain.resetEncoders();       // reset encoder values for starting point
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.doArcadeDrive(0, m_speed);     // turn robot at requested speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.doArcadeDrive(0, 0);     // Set motors to stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn
    return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_driveTrain.getLeftDistanceInch());
    double rightDistance = Math.abs(m_driveTrain.getRightDistanceInch());
    if (counter++ % 5 == 0) { System.out.println("**encoder: L/R: " + String.format("%.3f", leftDistance) +" ~ " + String.format("%.3f", rightDistance)); }
    return (leftDistance + rightDistance) / 2.0;
  }
}
