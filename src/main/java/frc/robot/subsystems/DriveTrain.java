// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.OmniWheel; // no need to import class in same directory
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final OmniWheel m_front;
  private final OmniWheel m_right;
  private final OmniWheel m_left;
  private final AHRS m_ahrs;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_front = new OmniWheel(1, "front", Constants.kWheelDiameterFt);
    m_right = new OmniWheel(2, "right", Constants.kWheelDiameterFt);
    m_left = new OmniWheel(3, "left", Constants.kWheelDiameterFt);
    m_ahrs = new AHRS(SPI.Port.kMXP);
    // The AHRS has not settled down on an angle yet when
    // the constructor for drive train is called.
    setZeroAngleDegrees(0.0); // assume 'front' wheel points 'ahead'
    beStill();
  }

  public void setZeroAngleDegrees(double degrees) {
    int nTries = 1;
    while (m_ahrs.isCalibrating()) { //wait to zero yaw if calibration is still running
      try {
        Thread.sleep(20);
        System.out.println("----calibrating gyro---- " + nTries);
      } catch (InterruptedException e) {

      }
      nTries++;
      if (nTries >= 50 && nTries%10==0) {
        System.out.println("Having trouble calibrating NavX");
      }
    }
    System.out.println("Setting angle adj to " + (-m_ahrs.getYaw()) + " + " + degrees + " after " + nTries + " attempts");
    m_ahrs.setAngleAdjustment(-m_ahrs.getYaw() + degrees);
  }

  public void setMotorsFractionPower(double front, double right, double left){
    m_front.setFractionPower(front);
    m_right.setFractionPower(right);
    m_left.setFractionPower(left);
  }

  /**
   * Set velocities of the motors in clicks per 100 ms.
   * @param front: velocity of front motor
   * @param right: velocity of right motor
   * @param left: velocity of left motor
   */
  public void setMotorVelocitiesNativeUnits(double front, double right, double left) {
    m_front.setVelocityNativeUnits(front);
    m_right.setVelocityNativeUnits(right);
    m_left.setVelocityNativeUnits(left);
  }

  public void setWheelVelocitiesFps(double frontFeetPerSecond, double rightFeetPerSecond, double leftFeetPerSecond){
    m_front.setVelocityFeetPerSecond(frontFeetPerSecond);
    m_right.setVelocityFeetPerSecond(rightFeetPerSecond);
    m_left.setVelocityFeetPerSecond(leftFeetPerSecond); 
  }

  /** spin robot using same power to all omniwheels
   *  @param fractionPower: number in [-1,1] saying what fraction of maximum power
   * to give each wheel.  Positive for clockwise, negative counterclockwise.
   * Useful for testing, but not recommended for general use.
   */
  public void spinFractionPower(double fractionPower) {
    setMotorsFractionPower(fractionPower, fractionPower, fractionPower);
  }

  /**
   * 
   * @param velocity: turn all wheels at this many clicks per 100 ms.
   *   This is useful for testing, but not recommended for general use.
   */
  public void spinNativeUnits(double velocity) {
    setMotorVelocitiesNativeUnits(velocity, velocity, velocity);
  }

  /**
   * 
   * @param degreesPerSecond: the robot itself (not a wheel) should turn this many degrees per second.
   * Positive degrees means clockwise, negative counterclockwise
   */
  public void spinDegreesPerSecond(double degreesPerSecond){
    double wheelVelocityFeetPerSecond = 2.0 * Math.PI * Constants.kCenterToWheelFt / 360. * degreesPerSecond;
    setWheelVelocitiesFps(wheelVelocityFeetPerSecond, wheelVelocityFeetPerSecond, wheelVelocityFeetPerSecond);
  }

  public void beStill() {
    setMotorsFractionPower(0.0, 0.0, 0.0);
  }

  /**
   * Make robot travel in a straight line
   * @param feetPerSecond: desired robot speed
   * @param radians: direction of travel in radians counterclockwise from straight ahead
   */
  public void setVelocityFpsRadians(double feetPerSecond, double radians){
    double frontFeetPerSecond = feetPerSecond * Math.cos( radians - 9./6. * Math.PI);
    double rightFeetPerSecond = feetPerSecond * Math.cos( radians - 5./6. * Math.PI);
    double leftFeetPerSecond = feetPerSecond * Math.cos( radians - 1./6. * Math.PI);
    setWheelVelocitiesFps(frontFeetPerSecond, rightFeetPerSecond, leftFeetPerSecond);
  }

  public void setVelocityFpsDegrees(double feetPerSecond, double degrees){
    setVelocityFpsRadians(feetPerSecond, -degrees / 180. * Math.PI);
  }

  public void setVelocityFpsDegrees_FieldOriented(double feetPerSecond, double degrees){
    double orientationDegrees = m_ahrs.getAngle();
    setVelocityFpsDegrees(feetPerSecond, degrees - orientationDegrees);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double angle = m_ahrs.getAngle();
    SmartDashboard.putNumber("Angle", angle);
  }
}
