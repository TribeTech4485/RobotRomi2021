// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Creates a new RomiGyro. */
public class RomiGyro extends SubsystemBase {
  
  private SimDouble m_simRateX;
  private SimDouble m_simRateY;
  private SimDouble m_simRateZ;
  private SimDouble m_simAngleX;
  private SimDouble m_simAngleY;
  private SimDouble m_simAngleZ;

  private double m_angleXOffset;
  private double m_angleYOffset;
  private double m_angleZOffset;

  // Seems like this is no longer needed.
  // As of the 2021.2.1 release the drift is more acceptable without manually correcting/
  // Previously I was getting about .5 angle drift per seconds which was affecting turning commands
  Timer timer = new Timer();
  public static double angleDriftPerSecX = 0; // 3.581;
  public static double angleDriftPerSecY = 0; //-0.149
  public static double angleDriftPerSecZ = 0; //0.555;

  /** Constructor to create a new RomiGyro. */
  public RomiGyro() {

    timer.start();

    SimDevice gyroSimDevice = SimDevice.create("Gyro:RomiGyro");
    if (gyroSimDevice != null) {
      gyroSimDevice.createBoolean("init", Direction.kOutput, true);
      m_simRateX = gyroSimDevice.createDouble("rate_x", Direction.kInput, 0.0);
      m_simRateY = gyroSimDevice.createDouble("rate_y", Direction.kInput, 0.0);
      m_simRateZ = gyroSimDevice.createDouble("rate_z", Direction.kInput, 0.0);

      m_simAngleX = gyroSimDevice.createDouble("angle_x", Direction.kInput, 0.0);
      m_simAngleY = gyroSimDevice.createDouble("angle_y", Direction.kInput, 0.0);
      m_simAngleZ = gyroSimDevice.createDouble("angle_z", Direction.kInput, 0.0);
    }
  }

  /**
   * Get the rate of turn in degrees-per-second around the X-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateX() {
    if (m_simRateX != null) {
      return m_simRateX.get();
    }
    return 0.0;
  }

  /**
   * Get the rate of turn in degrees-per-second around the Y-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateY() {
    if (m_simRateY != null) {
      return m_simRateY.get();
    }
    return 0.0;
  }

  /**
   * Get the rate of turn in degrees-per-second around the Z-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateZ() {
    if (m_simRateZ != null) {
      return m_simRateZ.get();
    }
    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around X-axis in degrees
   */
  public double getAngleX() {
    double driftXOffset = timer.get() * angleDriftPerSecX;
    if (m_simAngleX != null) {
      return m_simAngleX.get() - m_angleXOffset - driftXOffset;
    }
    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around Y-axis in degrees
   */
  public double getAngleY() {
    double driftYOffset = timer.get() * angleDriftPerSecY;
    if (m_simAngleY != null) {
      return m_simAngleY.get() - m_angleYOffset - driftYOffset;
    }
    return 0.0;
  }

  /**
   * Get the currently reported angle around the Z-axis.
   *
   * @return current angle around Z-axis in degrees
   */
  public double getAngleZ() {
    double driftZOffset = timer.get() * angleDriftPerSecZ;
    if (m_simAngleZ != null) {
      return m_simAngleZ.get() - m_angleZOffset - driftZOffset;
    }
    return 0.0;
  }

  /** Reset the gyro angles to 0. */
  public void reset() {
    timer.reset();
    if (m_simAngleX != null) {
      m_angleXOffset = m_simAngleX.get();
      m_angleYOffset = m_simAngleY.get();
      m_angleZOffset = m_simAngleZ.get();
    }
  }
}
