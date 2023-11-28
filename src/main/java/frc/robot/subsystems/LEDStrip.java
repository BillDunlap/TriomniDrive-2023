// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip extends SubsystemBase {
  public enum MotionType {
    kNoMotion,
    kRotateOut,
    kRotateIn
  }
  private AddressableLED m_ledStrip;
  private AddressableLEDBuffer m_ledBuffer;
  private MotionType m_motionType = MotionType.kNoMotion;
  private double m_delay = 0.1;
  private Timer m_timer; 
  /** Creates a new LEDStrip. */
  public LEDStrip(int pwmPort, int length) {
    m_ledStrip = new AddressableLED(pwmPort);
    m_ledStrip.setLength(length);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_timer = new Timer();
    turnAllOff();
    start();
  }
  /**
   * Turn on the lights
   */
  public void start(){
    m_ledStrip.start();
    m_timer.reset();
    m_timer.start();
  }
  /**
   * Turn off the lights?  Or just stop the motion?
   */
  public void stop(){
    turnAllOff();
    m_timer.reset();
    // m_ledStrip.stop();
  }
  /**
   * send data in buffer to strip
   */
  private void setData(){
    m_ledStrip.setData(m_ledBuffer);
  }
  /**
   * set all leds to dark.
   */
  public void turnAllOff(){
    for(int i = 0 ; i < m_ledBuffer.getLength() ; i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    setData();
  }
  /**
   * @param colors: A sequence of colors.  This sequence will be repeated to the length of the led strip,
   *                so it looks nicest if the number of colors evenly divides the length of the strip.
   */
  public void setRepeatingColorPattern(Color... colors){
     for(int i = 0 ; i < m_ledBuffer.getLength() ; i++){
      Color color = colors[i % colors.length];
      m_ledBuffer.setLED(i, color);
    }
    setData();
  }
  /**
   * @param colors: a sequence of colors to place at the start of the strip.  If the sequence is shorter
   * than the strip then the remaining leds on the strip will be turned off; if longer the extra color
   * entries will be ignored.
   */
  public void setColors(Color... colors){
    turnAllOff();
    int i = 0;
    for(Color color: colors){
      if (i < m_ledBuffer.getLength()){
        m_ledBuffer.setLED(i, color);
      } else {
        break;
      }
      i++;
    }
    setData();
  }
  /**
   * Rotate colors outward (away from the wired end of the strip).  Last color on strip will be copied to first.
   */
  public void movePatternOut(){
    Color previousColor = m_ledBuffer.getLED(0);
    for(int i = 1 ; i < m_ledBuffer.getLength() ; i++){
      Color currentColor = m_ledBuffer.getLED(i);
      m_ledBuffer.setLED(i, previousColor);
      previousColor = currentColor;
    }
    m_ledBuffer.setLED(0, previousColor);
    setData();
  }
  /**
   * Rotate colors inward (towards wired end of the strip).  First color on strip will be copied to the last.
   */
  public void movePatternIn(){
    Color nextColor = m_ledBuffer.getLED(m_ledBuffer.getLength()-1);
    for(int i = m_ledBuffer.getLength() - 2; i >= 0 ; i--){
      Color currentColor = m_ledBuffer.getLED(i);
      m_ledBuffer.setLED(i, nextColor);
      nextColor = currentColor;
    }
    m_ledBuffer.setLED(m_ledBuffer.getLength()-1, nextColor);
    setData();
  }

  public void setDelay(double delay){
    m_delay = delay;
  }
  public void setMotionType(MotionType motionType){
    m_motionType = motionType;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_motionType.equals(MotionType.kNoMotion) && m_timer.get() >= m_delay) {
      m_timer.reset();
      m_timer.start();
      switch(m_motionType){
        case kNoMotion:
          // do nothing
          break;
        case kRotateIn:
          movePatternIn();
          break;
        case kRotateOut:
          movePatternOut();
          break;
      }
    }
  }
}
