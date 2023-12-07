// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrip;
/**
 * Run a chase pattern on the LEDstrip
 */
public class ChaseLED extends CommandBase {
  LEDStrip m_ledStrip;
  double m_delay;
  Color[] m_colors;
  Timer m_timer;
  public enum Direction{
    kIn,
    kOut
  }
  Direction m_direction;
  /** Creates a new ChaseLED. */
  public ChaseLED(LEDStrip ledStrip, double delay, Direction direction, Color[] colors) {
    m_ledStrip = ledStrip;
    m_delay = delay;
    m_colors = colors;
    m_direction = direction;
    m_timer = new Timer();
    addRequirements(m_ledStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledStrip.setRepeatingColorPattern(m_colors);
    m_ledStrip.setMotionType(LEDStrip.MotionType.kNoMotion);
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= m_delay) {
      switch(m_direction){
        case kIn:
          m_ledStrip.movePatternIn();
          break;
        case kOut:
          m_ledStrip.movePatternOut();
          break;
      }
      m_timer.restart();
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
