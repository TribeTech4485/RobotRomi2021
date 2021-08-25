// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveCommand extends CommandBase {

    private final RomiDrivetrain m_driveTrain;
    private final DoubleSupplier m_leftSupplier;  // button value for left power 
    private final DoubleSupplier m_rightSupplier; // button value for right power
  
    private int counter = 4;    // for limiting display

  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(DoubleSupplier leftPower, DoubleSupplier rightPower) {

    this.m_driveTrain = RobotContainer.m_romiDrivetrain;
    this.m_leftSupplier = leftPower;
    this.m_rightSupplier = rightPower;
  
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

    // Drive robot using controller
    // Controller axis are measured from -1.0 to 1.0. -1.0 being all way down and +1.0 being all way up
    // Motor controllers also take values from -1.0 to 1.0. -1.0 being 100% power backwards, +1 100^ forward
    // Y axis of left stick moves the left side of the robot forward and backward
    // Y axis of right stick moves the right side of the robot forward and backward
    // double leftDrivePower = xController.getY(Hand.kLeft);  // Left Y axis
    // double rightDrivePower = xController.getY(Hand.kRight); // Right Y axis
    
    double leftValue = m_leftSupplier.getAsDouble();    // get values from controller
    double rightValue = m_rightSupplier.getAsDouble();
    m_driveTrain.arcadeDrive(leftValue, rightValue);    // do arcade drive

    counter++;  //*** only needed if limiting this display
    if (counter % 5 == 0) {
      System.out.println("DriveCommand left: "+String.format("%.4f", leftValue)+" right: "+String.format("%.4f", rightValue)); 
      SmartDashboard.putNumber("LeftDrive %", leftValue);
      SmartDashboard.putNumber("RightDrive %", rightValue);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;   // this is default command so should never end
  }
}
