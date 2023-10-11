// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.TuningVariables;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.DoubleArraySubscriber;

import edu.wpi.first.math.geometry.Transform3d;
import org.opencv.core.Point;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagPoseEstimate;

/** Get Apriltag information collected (and published on Network Tables)
 * by Raspberry Pi and arrange it for use by navigation commands. */

public class ApriltagInfo extends SubsystemBase {
  public class ApriltagRecord {
    public long m_timestamp;
    public boolean m_seen;
    public int m_id;
    public Transform3d m_transform3d;
    public Point m_center;
    ApriltagRecord(){
      m_seen = false;
      m_timestamp = -9876543210L; // an impossible value, well in the past
    }
    @Override
    public String toString() {
      return "Apriltag " + m_id + " (at " + (m_timestamp < 0 ? "Never" : m_timestamp) + ")" + (m_seen ? "seen" : "not seen");  
    }
    /* Apriltag coordinate system has z pointing forward,
     * x side to side, and y up and down.  Hence 'yaw' tells
     * if we have tipped over and 'pitch' gives the angle from
     * plane of apriltag to the camera.  Positive pitch means
     * we are to apriltag's right (hence must go left to be directly
     * in front of it).
     */
    public boolean wasSeen() {
      return m_seen;
    }
    public double getDistance() {
      return wasSeen() ? m_transform3d.getTranslation().getZ() : 0.0; // TODO: should use the 2 horizontal axes
    }
    public double getYaw() {
      return wasSeen() ? m_transform3d.getRotation().getZ() : 0.0;
    }
    public double getPitch() {
      return wasSeen() ? m_transform3d.getRotation().getY() : 0.0;
    }
    public double getRoll() {
      return wasSeen() ? m_transform3d.getRotation().getX() : 0.0;
    }
    public double getFrameX() {
      return wasSeen() ? m_center.x : -1.1;
    }
    public double getFrameY() {
      return wasSeen() ? m_center.y : -1.1;
    }
  }
  // https://www.ssontech.com/docs/SynthEyesUM_files/Choosing_an_AprilTag.html
  // says family 16h5 contains 30 distinct tags.
  private final int[] m_apriltagIdsOfInterest;
  private HashMap<Integer, ApriltagRecord> m_apriltagRecords = new HashMap<Integer, ApriltagRecord>();
  private final NetworkTableInstance m_instance;
  private HashMap<Integer, DoubleArraySubscriber> m_idPoseCenterSubscribers = new HashMap<Integer, DoubleArraySubscriber>();
  
  /** Create a Network Tables client to collect AprilTag information broadcast by the Raspberry Pi
   * @param teamNumber - used to locate the NT server
   * @param clientName - used to identify this client
   * @param apriltagIdsOfInterest - list of AprilTags that we wish to track
   */
  public ApriltagInfo(int teamNumber, String clientName, int[] apriltagIdsOfInterest) {
    m_instance = NetworkTableInstance.getDefault();
    m_instance.setServerTeam(teamNumber);
    m_instance.startClient4(clientName); // does server already exist?
    m_apriltagIdsOfInterest = apriltagIdsOfInterest;

    for(int id : m_apriltagIdsOfInterest) {
      m_idPoseCenterSubscribers.put(id, m_instance.getDoubleArrayTopic("/Apriltag/id_pose_center_" + id).subscribe(new double[]{}));
      m_apriltagRecords.put(id, new ApriltagRecord());
    }
  }

  public void updateRecordFromNetworkTables(int id){
    TimestampedDoubleArray tsr = m_idPoseCenterSubscribers.get(id).getAtomic(); // time + raw vector of 9 doubles
    double[] idPosCenter = tsr.value;
    ApriltagRecord apriltagRecord = getApriltagRecord(id);
    if (tsr.timestamp == apriltagRecord.m_timestamp) {
      // nothing new - do nothing.  This is a wierd case.
    } else if (idPosCenter.length < 9){
      // means this id was not seen
      apriltagRecord.m_seen = false;
    } else {
      if (idPosCenter[0] != id) {
        throw new RuntimeException("--- apriltag id mismatch: want " + id + ", but got" + idPosCenter[0]);
      }
      apriltagRecord.m_id = (int)(idPosCenter[0]);
      apriltagRecord.m_seen = true;
      apriltagRecord.m_transform3d = makeTransform3d(idPosCenter);
      apriltagRecord.m_center = makeCenter(idPosCenter);
      apriltagRecord.m_timestamp = tsr.timestamp;
      if (TuningVariables.debugLevel.get() >= 3) {
        SmartDashboard.putNumber("id", idPosCenter[0]);
      }
    }

  }

  private static Transform3d makeTransform3d(double[] idPosCenter){
    return new Transform3d(new Translation3d(idPosCenter[1], idPosCenter[2], idPosCenter[3]),
      new Rotation3d(idPosCenter[4], idPosCenter[5], idPosCenter[6]));
  }

  private static Point makeCenter(double[] idPosCenter){
    return new Point(idPosCenter[7], idPosCenter[8]);
  }

  public ApriltagRecord getApriltagRecord(int id) {
    return m_apriltagRecords.get(id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(int id : m_apriltagIdsOfInterest) {
      updateRecordFromNetworkTables(id);
    }
    ApriltagRecord apriltagRecord1 = getApriltagRecord(1);
    if (TuningVariables.debugLevel.get() >= 4) {
      SmartDashboard.putString("apriltag id 1", apriltagRecord1.toString());
    }
    double rToD = 180.0 / Math.PI;
    if (TuningVariables.debugLevel.get() >= 4) {
      SmartDashboard.putNumber("Yaw 1", apriltagRecord1.getYaw() * rToD);
      SmartDashboard.putNumber("Pitch 1", apriltagRecord1.getPitch() * rToD);
      SmartDashboard.putNumber("Roll 1", apriltagRecord1.getRoll() * rToD);
      SmartDashboard.putNumber("1 was seen", apriltagRecord1.wasSeen() ? 1.0 : 0.0);
      SmartDashboard.putNumber("1's frame x", apriltagRecord1.getFrameX());
    }
    //SmartDashboard.put("array0", getArray()[0]);
    //SmartDashboard.putNumberArray("rPi Array", getArray());
  }
}
