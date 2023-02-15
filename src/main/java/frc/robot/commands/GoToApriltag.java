// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ApriltagInfo;

/** Find the April Tag with the desired ID and go to it.
 * Consider it done when the robot is c. a meter from the
 * April Tag, directly in front of it, and facing it.
 * 
 * States:
 *  Searching: If the april tag is not visible (i.e., not
 *     in camera frame), then rotate slowly
 *     until either we see it or we have gone a full circle)
 *     If we go a full circle without seeing it, give up.
 *  Centering: If tag is seen, rotate until it is near center
 *     of camera frame.
 *  Translating: Move towards desired position.
 *  Done: We are close enough to our goal or have given up
 */

public class GoToApriltag extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final ApriltagInfo m_aprilTagInfo;
  private final int m_apriltagId;
  private enum State {
    kSEARCHING,
    kCENTERING,
    kTRANSLATING,
    kDONE
  }
  private State m_state;
  /** Creates a new GoToApriltag. */
  
  public GoToApriltag(int aprilTagId, DriveTrain driveTrain, ApriltagInfo aprilTagInfo) {
    m_driveTrain = driveTrain;
    m_aprilTagInfo = aprilTagInfo;
    m_apriltagId = aprilTagId;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.kSEARCHING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private double m_last_frameX=0.0;
  @Override
  public void execute() {
    ApriltagInfo.ApriltagRecord record = m_aprilTagInfo.getApriltagRecord(m_apriltagId);
    switch (m_state){
      case kSEARCHING:
        if (record.m_seen) {
          System.out.println("SEARCHING " + m_apriltagId + ": frameX=" + record.getFrameX());
          m_state = State.kCENTERING;
        } else {
          m_driveTrain.spinDegreesPerSecond(40.0);
        }
        break;
      case kCENTERING:
        double frameX = record.getFrameX();
        if (record.m_seen) {
          if (Math.abs(m_last_frameX - frameX) > 0.01) {
            System.out.println("CENTERING " + m_apriltagId + ": frameX=" + frameX);
            m_last_frameX = frameX;
          }
          if (Math.abs(frameX) <= 0.10) {
            System.out.println("DONE " + m_apriltagId + ": frameX=" + frameX);
            m_state = State.kDONE;
          } else {
            // TODO: spin rate should depend on distance to target (slower when farther away)
            m_driveTrain.spinDegreesPerSecond( Math.signum(frameX) * 25.0 );
          }
        } else {
          m_state = State.kSEARCHING;
        }
        break;
      case kTRANSLATING:
        break;
      case kDONE:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.beStill();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_state == State.kDONE;
  }
}
