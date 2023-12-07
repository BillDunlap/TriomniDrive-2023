// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrip;
/**
 * The default command for the LED strip
 */
public class DefaultLED extends CommandBase {
  private LEDStrip m_ledStrip;
  private Color[] m_chaseColors = new Color[]{
    Color.kWhite,
    Color.kRed, Color.kRed, Color.kRed, Color.kRed,
    Color.kWhite,
    Color.kGreen, Color.kGreen, Color.kGreen, Color.kGreen 
  };
  private Color[][] m_blinkColorArrays = new Color[][]{
    new Color[]{
      Color.kGreen, Color.kGreen, Color.kGreen, Color.kRed
    },
    new Color[]{
      Color.kRed, Color.kRed, Color.kRed, Color.kGreen
    }
  };
  private double m_chaseDelay = 0.1;
  private double[] m_blinkDelays = {0.2, 0.2};
  private int m_blinkIndex = 0;
  public enum MotionType {
    kNoMotion,
    kChaseOut,
    kChaseIn,
    kBlink
  }
  XboxController m_controller;
  MotionType m_motionType = MotionType.kNoMotion;
  Timer m_timer;
  public DefaultLED(LEDStrip ledStrip, XboxController controller){
    m_ledStrip = ledStrip;
    m_controller = controller;
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    checkBlinkData();
    m_timer = new Timer();
    m_timer.restart();
    m_ledStrip.start();
  }

  public void setBlinkData(double[] blinkDelays, Color[][] blinkColorArrays) {
    m_blinkDelays = blinkDelays;
    m_blinkColorArrays = blinkColorArrays;
    checkBlinkData();
  }
  private void checkBlinkData() {
    if (m_blinkDelays.length != m_blinkColorArrays.length){
      throw new Error("" + m_blinkDelays.length + " blink delays and " +
        m_blinkColorArrays.length + " color arrays given: the numbers must match.");
    }
    if (m_blinkDelays.length == 0) {
      throw new Error("0 blink entries given: the number must be positive.");
    }
    m_blinkIndex = m_blinkIndex % m_blinkDelays.length;
  }
  public void doBlink(){
    m_motionType = MotionType.kBlink;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    readController();
    doAction();
  }
  public void doAction() {
    switch(m_motionType){
      case kChaseIn:
      case kChaseOut:
        if (m_timer.get() > m_chaseDelay) {
          if (m_motionType == MotionType.kChaseIn) {
            m_ledStrip.movePatternIn();
          } else {
            m_ledStrip.movePatternOut();
          }
          m_timer.restart();
        }
        break;
      case kBlink:
        if (m_timer.get() > m_blinkDelays[m_blinkIndex]) {
          m_ledStrip.setRepeatingColorPattern(m_blinkColorArrays[m_blinkIndex]);
          m_blinkIndex = (m_blinkIndex + 1) % m_blinkDelays.length;
          m_timer.restart();
        }
        break;
      case kNoMotion:
        break;
    }
  }
  private void readController() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
