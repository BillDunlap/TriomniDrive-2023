// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ApriltagInfo;
import frc.robot.Vector2D;

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
  private final Vector2D m_targetPosition;
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
    m_targetPosition = new Vector2D(0.0, 0.75, Vector2D.ConstructorType.kRadiansCcwDistance);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.kSEARCHING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private double m_last_frameX=0.0; // used only to limit debug printing

  private double m_desiredDistanceFromTarget = 0.66; // meters, c. 2 feet
  private double m_distanceTolerance = 0.05; // c. 2 inches
  private double m_angularTolerance = 0.1; // c. +-5.7 degrees
  private boolean isDone(ApriltagInfo.ApriltagRecord record) {
    // TODO: use Vector2D 
    return record.m_seen && 
    Math.abs(record.getDistance() - m_desiredDistanceFromTarget) <= m_distanceTolerance &&
    // positive pitch means we are to apriltag's right
    Math.abs(record.getPitch() - 0.0) <= m_angularTolerance &&
    Math.abs(record.getFrameX()) <= 0.05 ;
  }

  private void setState(State state){
    System.out.println("/_\\ " + m_state + " -> " + state);
    m_state = state;
  }
  
  @Override
  public void execute() {
    ApriltagInfo.ApriltagRecord record = m_aprilTagInfo.getApriltagRecord(m_apriltagId);
    switch (m_state){
      case kSEARCHING:
        if (record.m_seen) {
          System.out.println("found " + m_apriltagId + ": frameX=" + record.getFrameX());
          setState(State.kCENTERING);
        } else if (isDone(record)){
          setState(State.kDONE);
        } else {
          m_driveTrain.spinDegreesPerSecond(20.0);
        }
        break;
      case kCENTERING:
        double frameX = record.getFrameX();
        if (!record.m_seen) {
          setState(State.kSEARCHING);
        } else if (isDone(record)) {
          setState(State.kDONE);
        } else if (Math.abs(frameX) <= 0.05) {
          setState(State.kTRANSLATING);
        } else {
          if (Math.abs(m_last_frameX - frameX) > 0.01) { // for debug only
            System.out.println("CENTERING " + m_apriltagId + ": frameX=" + frameX);
            m_last_frameX = frameX;
          }
          // TODO: spin rate should depend on distance to target (slower when farther away)
          m_driveTrain.spinDegreesPerSecond( Math.signum(frameX) * 20.0 );
        }
        break;
      case kTRANSLATING:
        if (!record.m_seen) {
          setState(State.kSEARCHING);
        } else if (isDone(record)) {
          setState(State.kDONE);
        } else if (Math.abs(record.getFrameX()) >= 0.2) {
          setState(State.kCENTERING);
        } else {
          Vector2D currentPosition = new Vector2D(record.getPitch(), record.getDistance(), Vector2D.ConstructorType.kRadiansCcwDistance);
          Vector2D toTarget = currentPosition.vectorTo(m_targetPosition);
          double robotRelativeRadiansCcw = toTarget.getRadiansCcw() - (Math.PI + currentPosition.getRadiansCcw());
          // System.out.println("TRANSLATING: " + robotRelativeRadiansCcw/Math.PI + " pi radians");
          m_driveTrain.setVelocityFpsRadians(2.5, robotRelativeRadiansCcw);
        }
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
