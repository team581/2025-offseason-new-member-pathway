// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {

  /** Driver Controller is on port 0 */
  private final CommandXboxController driverController = new CommandXboxController(0);

  /** Intake Roller Motor has ID of 25, and is on CANbus with name 581CANivore */
  private final TalonFX intakeMotor = new TalonFX(25, "581CANivore");

  /** Called once at the beginning of the robot program. */
  public Robot() {

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

  }

}
