// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerRumbler extends SubsystemBase {
  private XboxController m_controller;
  /** Creates a new ControllerRumbler. */
  public ControllerRumbler(XboxController controller) {
    m_controller = controller;
  }

  public void setRumble(double left, double right){
    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, left);
    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, right);
  }

  public void setRumble(double both){
    setRumble(both, both);
  }

  public void beQuiet(){
    setRumble(0.0);
  }
}
