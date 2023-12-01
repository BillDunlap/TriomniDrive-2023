// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.MotionType;

public class BlinkLED extends CommandBase {
  private LEDStrip m_ledStrip;
  private double[] m_durations;
  private Color[][] m_colorsArray;
  private int m_which; // index of colorArray to display now
  private Timer m_timer; 
  /** 
   * Creates a new BlinkLED command.  This will cycle through different color patterns on the LED strip.
   * @param ledStrip: the robot's LED strip
   * @param durations: an array the length of colorsArray giving the number of seconds to display each color array
   * @param colorsArray: an array of arrays of Colors 
   * */
  public BlinkLED(LEDStrip ledStrip, double[]durations, Color[][] colorsArray) {
    m_ledStrip = ledStrip;
    m_durations = durations;
    m_colorsArray = colorsArray;
    m_timer = new Timer();
    if (m_durations.length != m_colorsArray.length) {
      throw new Error("" + m_durations.length + " durations and " + m_colorsArray.length + " color arrays given - lengths must match");
    }
    addRequirements(m_ledStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledStrip.setMotionType(MotionType.kNoMotion);
    m_which = -1;
    startNewColorPattern();
  }

  private void startNewColorPattern()
  {
    m_which = (m_which+1) % m_durations.length;
    m_ledStrip.setRepeatingColorPattern(m_colorsArray[m_which]);
    m_ledStrip.start();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= m_durations[m_which]) {
      startNewColorPattern();
    }
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
