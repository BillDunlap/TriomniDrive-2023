// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.LEDStrip.MotionType;

/**
 * Make a repeating Christmas-themed pattern and make it move along the led strip 
 * every 'delay' seconds.
 */
public class XmasPattern extends CommandBase {
  private LEDStrip m_ledStrip;
  private double m_delay;
  private LEDStrip.MotionType m_motionType;
  /** Creates a new XmasPattern. */
  public XmasPattern(LEDStrip ledStrip, double delay, MotionType motionType) {
    m_ledStrip = ledStrip;
    m_motionType = motionType;
    m_delay = delay;
    addRequirements(m_ledStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledStrip.setDelay(m_delay);
    m_ledStrip.setMotionType(m_motionType);
    m_ledStrip.setRepeatingColorPattern(
      Color.kPink,
      Color.kRed, Color.kRed, Color.kRed, Color.kRed,
      Color.kPink,
      Color.kLightSlateGray,
      Color.kLightGreen,
      Color.kGreen, Color.kGreen, Color.kGreen, Color.kGreen,
      Color.kLightGreen,
      Color.kLightSlateGray);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledStrip.turnAllOff();
    m_ledStrip.setMotionType(LEDStrip.MotionType.kNoMotion);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
