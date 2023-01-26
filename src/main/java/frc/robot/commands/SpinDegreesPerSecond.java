// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SpinDegreesPerSecond extends CommandBase {
  DriveTrain m_driveTrain;
  double m_degreesPerSecond;
  /** Creates a new SpinDegreesPerSecond. */
  public SpinDegreesPerSecond(DriveTrain driveTrain, double degreesPerSecond) {
    m_driveTrain = driveTrain;
    m_degreesPerSecond = degreesPerSecond;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.spinDegreesPerSecond(m_degreesPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.beStill();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // We expect caller to use .until(predicate) or .withTimeout(seconds)
  }
}
