// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SpinNativeUnits extends CommandBase {
  DriveTrain m_driveTrain;
  double m_velocity;
  /** Creates a new SpinNativeUnits.
   * @param driveTrain: an OmniWheel drivetrain
   * @param velocity: velocity in native units (ticks per 100 milliseconds)
   */
  public SpinNativeUnits(DriveTrain driveTrain, double velocity) {
    m_driveTrain = driveTrain;
    m_velocity = velocity;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.spinNativeUnits(m_velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.beStill();
  }

  // The command never ends: decorate it with withTimeout() or until(predicate function)
  @Override
  public boolean isFinished() {
    return false;
  }
}
