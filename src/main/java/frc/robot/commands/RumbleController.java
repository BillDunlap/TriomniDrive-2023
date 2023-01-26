// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControllerRumbler;

public class RumbleController extends CommandBase {
  ControllerRumbler m_controllerRumbler;
  double m_left; // intensity of rumble of left-hand rumbler
  double m_right; //                   ... right ...
  /** Creates a new RumbleController. */
  public RumbleController(ControllerRumbler controllerRumbler, double left, double right) {
    m_left = left;
    m_right = right;
    m_controllerRumbler = controllerRumbler;
    /* Note that other subsystems may invoke rumbler, but not
       through a command, so their settings may interfere with ones
       set in this command */
    addRequirements(m_controllerRumbler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controllerRumbler.setRumble(m_left, m_right);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Would not need this if other subsystems didn't also set rumble level
    m_controllerRumbler.setRumble(m_left, m_right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controllerRumbler.setRumble(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // we expect this command to be decorated with .withTimeout(seconds) or .until(predicate)
  }
}
