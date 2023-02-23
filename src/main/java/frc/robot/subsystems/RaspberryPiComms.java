// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.TuningVariables;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleArraySubscriber;
/** A class to test the use of Network Table to read
 * from a Raspberry PI which is running the following python code.
 * This should not be invoked except while developing NT code.
 * 
import time
import ntcore

# To see messages from networktables, you must setup logging
import logging
logging.basicConfig(level=logging.DEBUG)

instance = ntcore.NetworkTableInstance.getDefault()
instance.setServerTeam(4173)
instance.startClient4("rPi")

table = instance.getTable("/SmartDashboard")

testTopic = table.getDoubleTopic("test") # or instance.getDoubleTopic("/SmartDashboard/test")
testPublisher = testTopic.publish()
testPublisher.setDefault(-666.666)

arrayTopic = table.getDoubleArrayTopic("array")
arrayPublisher = arrayTopic.publish()
arrayPublisher.setDefault([])

angleTopic = instance.getDoubleTopic("/SmartDashboard/Angle")
angleSubscriber = angleTopic.subscribe(-123.456)

k = 0.0
while True:
    time.sleep(3)
    k += 17.0
    print(k)
    testPublisher.set(k)
    print(angleSubscriber.get())
    # make up variable length array to publish
    if k%2 == 0:
        arrayPublisher.set([k+1.0, k+2.0])
    else:
        arrayPublisher.set([k+1.25, k+2.25, k+3.25, k+4.25, k+5.25])
 */


public class RaspberryPiComms extends SubsystemBase {
  private final NetworkTableInstance m_instance;
  private final DoubleTopic m_testTopic;
  private final DoubleSubscriber m_testSubscriber;
  private final DoubleArrayTopic m_arrayTopic;
  private final DoubleArraySubscriber m_arraySubscriber;

  /** Creates a new RaspberryPiComms. */
  public RaspberryPiComms(int teamNumber, String clientName) {
    m_instance = NetworkTableInstance.getDefault();
    m_instance.setServerTeam(teamNumber);
    m_instance.startClient4(clientName); // does server already exist?
    m_testTopic = m_instance.getDoubleTopic("/SmartDashboard/test");
    m_testSubscriber = m_testTopic.subscribe(-1.0);
    m_arrayTopic = m_instance.getDoubleArrayTopic("/SmartDashboard/array");
    m_arraySubscriber = m_arrayTopic.subscribe(new double[]{-321.654});
  }

  public double getTest(){
    return m_testSubscriber.get();
  }

  public double[] getArray(){
    return m_arraySubscriber.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (TuningVariables.debugLevel.get() >= 3) {
      SmartDashboard.putNumber("RaspberryPiComms", getTest());
      SmartDashboard.putNumber("array0", getArray()[0]);
      SmartDashboard.putNumberArray("rPi Array", getArray());
    }
  }
}
