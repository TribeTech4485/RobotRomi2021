// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveCommandArcade extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_speedSupplier;  // button value for speed forward / backward 
  private final DoubleSupplier m_rotationSupplier; // button value for rotation

  private int counter = 50; // for limiting display

  /**
   * Creates a new DriveCommandArcade.
   */
  public DriveCommandArcade(DoubleSupplier speed, DoubleSupplier rotation) {

    this.m_driveTrain = RobotContainer.m_driveTrain;    // get driveTrain object from RobotContainer
    this.m_speedSupplier = speed;
    this.m_rotationSupplier = rotation;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Drive robot with arcade drive using controller
    // Controller axis are measured from -1.0 to 1.0. -1.0 being all way down and +1.0 being all way up
    // Y axis of left stick moves the robot forward and backward
    // Y axis of right stick rotates the robot clockwise / counter clockwise
    
    double xSpeed = -m_speedSupplier.getAsDouble();      // using tank drive, WPILib automatically inverts, so need neg here
    double zRotation = m_rotationSupplier.getAsDouble();
 
    if (counter++ % 50 == 0) { 	//*** only needed if limiting this display
      SmartDashboard.putNumber("speed %", xSpeed);
      SmartDashboard.putNumber("rotation %", zRotation);
    }

    if (zRotation >= 0.60 || zRotation <= -0.60) {
      zRotation *= 0.75;   // Romi sensitive to fast turns so reduce if needed 
    }
    m_driveTrain.doArcadeDrive(xSpeed, zRotation);      // set motors to requested speed and rotation
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;   // this command should not end
  }
}
