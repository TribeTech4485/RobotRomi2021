// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_xaxisSpeedSupplier;
  private final DoubleSupplier m_zaxisRotateSupplier;

  private int counter = 4;    // for limiting display

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      DoubleSupplier xaxisSpeedSupplier,
      DoubleSupplier zaxisRotateSuppplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    double xSpeed = 0.7*m_xaxisSpeedSupplier.getAsDouble();    // get values from controller
    double zRotate = 0.5*m_zaxisRotateSupplier.getAsDouble();
    m_drivetrain.arcadeDrive( xSpeed, zRotate);           // do arcade drive 

    counter++;  //*** only needed if limiting this display
    if (counter % 5 == 0) {
      System.out.println("arcadeDrive xSpeed: "+String.format("%.4f", xSpeed)+" rotate: "+String.format("%.4f", zRotate)); 
      SmartDashboard.putNumber("xSpeed %", xSpeed);
      SmartDashboard.putNumber("zRotate %", zRotate);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
