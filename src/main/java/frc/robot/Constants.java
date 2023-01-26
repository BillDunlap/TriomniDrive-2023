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
    public static class TalonSRX {
      /**
       * @param kTalonTimoutMs: timeout (in milliseconds) for TalonSRX config*() calls
        */
      public static final int kTimeoutMs = 20; 
      /**
       * @param kDriveTrainPIDLoopIndex: which of the Talon's PID slots to use on the drivetrain
       */
      public static final int kDriveTrainPIDLoopIndex = 0;
    }
    /**
     * @param kWheelDiameterFt: diameter of drive wheels in feet.
     */
    public static final double kWheelDiameterFt = 8.0 / 12.0; // 8 inch omniwheels
    /**
     * @param kCenterToWheelFt: horizontal distance from center of robot to each wheel 
     */
    public static final double kCenterToWheelFt = 18.0 / 12.0; // 18 inches
}