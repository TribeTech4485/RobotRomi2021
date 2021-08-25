// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {

        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterInch = 2.75591;    // 70 mm for Romi

        public static final double kToleranceDegrees = 2.0;  //indicates how close to "on target" acceptable
    }

    public static final class OIConstants {
        // XBox controller buttons
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

    }
}
