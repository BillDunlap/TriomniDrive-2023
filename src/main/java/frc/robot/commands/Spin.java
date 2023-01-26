// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Spin extends CommandBase {
  private final DriveTrain m_driveTrain;
  private double m_fractionPower;
  /** Creates a new Spin.
   * @param driveTrain: the drivetrain consisting of 3 omniwheels
   * @param fractionPower: how much power to put into the spin, positive for
   *        clockwise, negative counter-clockwise.
   */
  public Spin(DriveTrain driveTrain, double fractionPower) {
    m_fractionPower = fractionPower;
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.spinFractionPower(m_fractionPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.beStill();;
  }

  // We expect this command to be decorated with withTimeout() or until()
  @Override
  public boolean isFinished() {
    return false; // we expect user will add a decorator for a timeout or wait until button is released
  }
}
