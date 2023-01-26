// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GoStraight extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final double m_feetPerSecond;
  private final double m_degrees;
  /** Creates a new GoStraight. */
  public GoStraight(DriveTrain driveTrain, double feetPerSecond, double degrees) {
    m_driveTrain = driveTrain;
    m_feetPerSecond = feetPerSecond;
    m_degrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setVelocityFpsDegrees(m_feetPerSecond, m_degrees);
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
    return false; // expect programmer to decorate this with .until(predicate)
  }
}
