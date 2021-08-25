// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  // The Romi has left and right motors set to PWM channels 0 and 1 respectively 
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private int counter1 = 20;
  private int counter2 = 20;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    //m_leftMotor.setInverted(true);
    //m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterInch) / DriveConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterInch) / DriveConstants.kCountsPerRevolution);
  
    m_diffDrive.setSafetyEnabled(false);  // helps avoid 'differential drive not updated often enough'
  
    // reset encoders to 0 and reset gyro
    resetEncoders();
    m_gyro.reset();
  }

  public void doTankDrive(double leftSpeed, double rightSpeed) {
    if (counter1++ % 20 == 0) { 
      System.out.println("** tank drive: L/R: " + String.format("%.3f", leftSpeed) +" ~ " + String.format("%.3f", rightSpeed)); 
    }
    leftSpeed *= 0.75;    // reduce speed for testing
    rightSpeed *= 0.75;
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void doArcadeDrive(double xaxisSpeed, double zaxisRotate) {
    if (counter2++ % 20 == 0) {
      System.out.println(" **arcadeDrive:  xSpeed: " + String.format("%.4f", xaxisSpeed) + " zRotate: " + String.format("%.4f", zaxisRotate));
    }
    double xaxisTempS = xaxisSpeed * 0.75;  // reduce speed for testing
    double zaxisTempR = zaxisRotate * 0.75;
    m_diffDrive.arcadeDrive(xaxisTempS, zaxisTempR);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  public double getHeadingAngle() {
    return Math.IEEEremainder(getGyroAngleZ(), 360.0);
  }

  /** Reset the gyro */
  public void resetGyro() {
    m_gyro.reset();
    System.out.println("**after reset current gyro AngleZ: " + String.format("%.4f", getGyroAngleZ()));
  }

  public void printGyroOffsets() {
    System.out.println(MessageFormat.format("Gyro offsets  time: {0}  angleX: {1}  angleY: {2}  angleZ: {3}  rateX: {4}  rateY: {5}  rateZ: {6}", Timer.getFPGATimestamp(), m_gyro.getAngleX(),  m_gyro.getAngleY(),  m_gyro.getAngleZ(), m_gyro.getRateX(), m_gyro.getRateY(), m_gyro.getRateZ()));
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    System.out.println("**in driveTrain stop");
    doTankDrive(0.0, 0.0);
  }
}
