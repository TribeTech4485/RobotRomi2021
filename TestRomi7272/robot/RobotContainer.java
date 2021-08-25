// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commandgroups.DriveASquare;
import frc.robot.commandgroups.AutoSpinContinuous;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForwardInches;
import frc.robot.commands.DriveForwardInchesGyroStraightPID; 
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveTurnDegrees;
import frc.robot.commands.DriveTurnDegreesGyro;
import frc.robot.commands.DriveTurnDegreesGyroPID;
import frc.robot.commands.DriveTurnNetworkTableAnglePID;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; 

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private static final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.OUTPUT);

  // Controller & Joysticks
  private static final XboxController xController = new XboxController(0);
  //private static final Joystick auxController = new Joystick(0);
  //private static final Joystick keyboard = new Joystick(1);

  // The network table entries used across commands
  private final NetworkTableEntry visionAngle;
  private final NetworkTableEntry visionObjects;

  //Commands
  private final DriveForwardTimed forwardTimed = new DriveForwardTimed(m_romiDrivetrain, 4, .75);
  //private final DriveForwardTimed backwardTimed = new DriveForwardTimed(m_romiDrivetrain, 6, -.75);
  private final DriveForwardInches forward12in = new DriveForwardInches(m_romiDrivetrain, 12, .75);
  private final DriveForwardInches backward12in = new DriveForwardInches(m_romiDrivetrain, 12, -.75);
  private final DriveTurnDegrees right90degrees = new DriveTurnDegrees(m_romiDrivetrain, 90, .75);
  private final DriveTurnDegrees left90degrees = new DriveTurnDegrees(m_romiDrivetrain, -90, .75);
  private final DriveASquare squareDrive = new DriveASquare(m_romiDrivetrain);
  private final DriveTurnDegreesGyro gyroTurnRight = new DriveTurnDegreesGyro(m_romiDrivetrain, 45, .60);
  private final DriveTurnDegreesGyro gyroTurnLeft = new DriveTurnDegreesGyro(m_romiDrivetrain, -45, .60);
  private final DriveTurnDegreesGyroPID gyroPidTurnRightOnce = new DriveTurnDegreesGyroPID(m_romiDrivetrain, 90, .60, false);
  private final DriveTurnDegreesGyroPID gyroPidTurnLeftOnce = new DriveTurnDegreesGyroPID(m_romiDrivetrain, -90, .60, false);
  //private final DriveTurnDegreesGyroPID gyroPidTurnRightCont = new DriveTurnDegreesGyroPID(m_romiDrivetrain, 90, .60, true);
  private final DriveForwardInchesGyroStraightPID driveForwardInchesGyro = new DriveForwardInchesGyroStraightPID(m_romiDrivetrain, 18, .75);
  private final DriveTurnNetworkTableAnglePID driveTurnNetworkTable;
  private final AutoSpinContinuous autoSpinContinuous = new AutoSpinContinuous(m_romiDrivetrain);

  //set up autonomous chooser
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Get the network table entries used in the robot program.
    // In this example I'm using https://www.chiefdelphi.com/t/romi-vision/390540.
    // Just upload the jar from the build directory on github into http://wpilibpi.local/ as the Vision Application.
    // I don't have reflective tape on hand and tested (not ideal) with a piece of printer paper.
    // Done in constructor because NetworkTableEntry is just a reference to the entry, use get method in commands to retrieve values.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable visionTargetData = inst.getTable("targetData");
    this.visionAngle = visionTargetData.getEntry("drCorr");
    this.visionObjects = visionTargetData.getEntry("numberImages");
    
    // Instantiate commands using network table here (to keep all fields as finals)
    this.driveTurnNetworkTable = new DriveTurnNetworkTableAnglePID(m_romiDrivetrain, visionAngle, visionObjects, .7, true);

    //Default Commands
    //private final DefaultDrive defaultDrive = new DefaultDrive(xController, m_romiDrivetrain);
    m_romiDrivetrain.setDefaultCommand(new DriveCommand(() -> xController.getY(Hand.kLeft), () -> xController.getY(Hand.kRight)));

    // load autonomous commands in autochooser and put on dashboard
    autoChooser.setDefaultOption("Forward 18in GyroPID", driveForwardInchesGyro );
    autoChooser.addOption("Forward 12in", forward12in );
    autoChooser.addOption("Backward 12in", backward12in );
    autoChooser.addOption("Drive Timed 4s", forwardTimed );
    autoChooser.addOption("Turn Right 90", right90degrees );
    autoChooser.addOption("Turn Left 90", left90degrees );
    autoChooser.addOption("Drive Square", squareDrive );
    autoChooser.addOption("Turn Right 45 Gyro", gyroTurnRight );
    autoChooser.addOption("Turn Left 45 Gyro", gyroTurnLeft );
    autoChooser.addOption("Turn Right 90 GyroPID", gyroPidTurnRightOnce );
    autoChooser.addOption("Turn Left 90 GyroPID", gyroPidTurnLeftOnce );
    autoChooser.addOption("Forward 18in GyroPID", driveForwardInchesGyro );
    SmartDashboard.putData("Auto Choices", autoChooser);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Xbox 360 controller button bindings
    final JoystickButton xButtonA = new JoystickButton(xController, 1);
    final JoystickButton xButtonB = new JoystickButton(xController, 2);
    final JoystickButton xButtonX = new JoystickButton(xController, 3);
    final JoystickButton xButtonY = new JoystickButton(xController, 4);
    final JoystickButton xBumperL = new JoystickButton(xController, 5);
    final JoystickButton xBumperR = new JoystickButton(xController, 6);
    final JoystickButton xButtonBack = new JoystickButton(xController, 7);
    final JoystickButton xButtonStart = new JoystickButton(xController, 8);
    //xButtonA.whenPressed(forward12in);
    xButtonA.whenPressed(new DriveForwardInches(m_romiDrivetrain, 12, .75));
    xButtonB.whenPressed(right90degrees);
    xButtonX.whenPressed(left90degrees);
    xButtonY.whenPressed(backward12in);
    xBumperL.whenPressed(gyroTurnLeft);
    xBumperR.whenPressed(gyroTurnRight);
    xButtonBack.whenPressed(forwardTimed);
    xButtonStart.whenPressed(autoSpinContinuous);

    /*
    // Example of how to use the Romi onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(() -> m_onboardIO.setYellowLed(true))
        .whenInactive(() -> m_onboardIO.setYellowLed(false));
    */
    
    /*
    Button onboardButtonB = new Button(m_onboardIO::getButtonBPressed);
    onboardButtonB
        .whenActive(() -> m_onboardIO.setRedLed(true))
        .whenInactive(() -> m_onboardIO.setRedLed(false));
    */

    /*
    // Logitech button bindings
    final JoystickButton auxButtonA = new JoystickButton(auxController, 1);
    final JoystickButton auxButtonB = new JoystickButton(auxController, 2);
    final JoystickButton auxButtonX = new JoystickButton(auxController, 3);
    final JoystickButton auxButtonY = new JoystickButton(auxController, 4);
    final JoystickButton auxBumperL = new JoystickButton(auxController, 5);
    final JoystickButton auxBumperR = new JoystickButton(auxController, 6);
    auxButtonA.whenPressed(forward12in);
    auxButtonB.whenPressed(right90degrees);
    auxButtonX.whenPressed(left90degrees);
    auxButtonY.whenPressed(backward12in);
    auxBumperL.whenPressed(gyroTurnLeft);
    auxBumperR.whenPressed(gyroTurnRight);
    */

    /*
        //// XBox controller buttons
        public static final int kXBoxButtonA = 1;
        public static final int kXBoxButtonB = 2;
        public static final int kXBoxButtonX = 3;
        public static final int kXBoxButtonY = 4;
        public static final int kXBoxBumperLeft = 5;
        public static final int kXBoxBumperRight = 6;
        public static final int kXBoxButtonBack = 7;
        public static final int kXBoxButtonStart = 8;
        public static final int kXBoxStickLeft = 9;
        public static final int kXBoxStickRight = 10;

        // Logitech Gamepad axis
        public static final int kLogiTechAxisLeftStickX = 1;
        public static final int kLogiTechAxisLeftStickY = 2;
        public static final int kLogiTechAxisTriggers = 3; // left trigger only=-1.0, right only=1.0, both=0.0
        public static final int kLogiTechAxisRightStickX = 4;
        public static final int kLogiTechAxisRightStickY = 5;
        public static final int kLogiTechAxisDpad = 6;

        // Logitech Gamepad buttons
        public static final int kLogiTechButtonA = 1; // Bottom Button
        public static final int kLogiTechButtonB = 2; // Right Button
        public static final int kLogiTechButtonX = 3; // Left Button
        public static final int kLogiTechButtonY = 4; // Top Button
        public static final int kLogiTechBumperLeft = 5; // on front of controller
        public static final int kLogiTechBumperRight = 6;
        public static final int kLogiTechButtonBack = 7; // Small button top left
        public static final int kLogiTechButtonStart = 8; // Small button top right
        public static final int kLogiTechStickLeft = 9; // on front of controller
        public static final int kLogiTechStickRight = 10;
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // get command run in autonomous
    Command autoSelected = autoChooser.getSelected();
    return autoSelected;
    //return forward12in;
    //return squareDrive;
  }

  public Subsystem getDrivetrain() {
    return m_romiDrivetrain;
  }

  //public Command getDefaultDrive() {
  //  return defaultDrive;
  //}

  public void printGyroOffsets() {
    m_romiDrivetrain.printGyroOffsets();
  }

}
