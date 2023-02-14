// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
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
  private final int m_maxApriltagId = 30;
  private ApriltagRecord[] m_apriltagRecords = new ApriltagRecord[m_maxApriltagId];
  private final NetworkTableInstance m_instance;
  private final DoubleArraySubscriber[] m_idPoseCenterSubscriber = new DoubleArraySubscriber[m_maxApriltagId];
  
  /** Creates a new RaspberryPiComms. */
  public ApriltagInfo(int teamNumber, String clientName) {
    m_instance = NetworkTableInstance.getDefault();
    m_instance.setServerTeam(teamNumber);
    m_instance.startClient4(clientName); // does server already exist?

    for(int id=1; id<=m_maxApriltagId; id++) {
      m_idPoseCenterSubscriber[id-1] = m_instance.getDoubleArrayTopic("/Apriltag/id_pose_center_" + (id)).subscribe(new double[]{});
      m_apriltagRecords[id-1] = new ApriltagRecord();
    }
  }

  public void updateRecordFromNetworkTables(int id){
    TimestampedDoubleArray tsr = m_idPoseCenterSubscriber[id-1].getAtomic(); // time + raw vector of 9 doubles
    double[] idPosCenter = tsr.value;
    if (tsr.timestamp == m_apriltagRecords[id-1].m_timestamp) {
      // nothing new - do nothing.  This is a wierd case.
    } else if (idPosCenter.length < 9){
      // means this id was not seen
      m_apriltagRecords[id-1].m_seen = false;
    } else {
      if (idPosCenter[0] != id) {
        throw new RuntimeException("--- apriltag id mismatch: want " + id + ", but got" + idPosCenter[0]);
      }
      m_apriltagRecords[id-1].m_id = (int)(idPosCenter[0]);
      m_apriltagRecords[id-1].m_seen = true;
      m_apriltagRecords[id-1].m_transform3d = makeTransform3d(idPosCenter);
      m_apriltagRecords[id-1].m_center = makeCenter(idPosCenter);
      m_apriltagRecords[id-1].m_timestamp = tsr.timestamp;
      SmartDashboard.putNumber("id", idPosCenter[0]);
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
    return m_apriltagRecords[id-1];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(int id=1 ; id <= m_maxApriltagId ; id++) {
      updateRecordFromNetworkTables(id);
    }
    ApriltagRecord apriltagRecord1 = getApriltagRecord(1);
    SmartDashboard.putString("apriltag id 1", apriltagRecord1.toString());
    double rToD = 180.0 / Math.PI;
    SmartDashboard.putNumber("Yaw 1", apriltagRecord1.getYaw() * rToD);
    SmartDashboard.putNumber("Pitch 1", apriltagRecord1.getPitch() * rToD);
    SmartDashboard.putNumber("Roll 1", apriltagRecord1.getRoll() * rToD);
    SmartDashboard.putNumber("was seen", apriltagRecord1.wasSeen() ? 1.0 : 0.0);
    SmartDashboard.putNumber("frame x", apriltagRecord1.getFrameX());
    //SmartDashboard.put("array0", getArray()[0]);
    //SmartDashboard.putNumberArray("rPi Array", getArray());
  }
  
}