// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
  /** Driver Controller is on port 0 */
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(driverController);

  /** Intake Deploy Motor has ID of 20, and is on CANbus with name 581CANivore */
  private final TalonFX deployMotor = new TalonFX(20, "581CANivore");

  /** Intake Roller Motor has ID of 25, and is on CANbus with name 581CANivore */
  private final TalonFX intakeMotor = new TalonFX(25, "581CANivore");

  /** Intake CANDi has ID of 26, and is on CANbus with name 581CANivore */
  private final CANdi intakeCandi = new CANdi(26, "581CANivore");

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeCandi);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    intakeMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(20)
                        .withSupplyCurrentLimit(20))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));
    deployMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withFeedback(
                    new FeedbackConfigs()
                        .withSensorToMechanismRatio((50.0 / 8.0) * (50.0 / 18.0) * (40.0 / 10.0)))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(20)
                        .withSupplyCurrentLimit(20))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if (driverController.getLeftTriggerAxis() > 0.5) {
      // Left trigger to intake
      intakeSubsystem.intake();
    } else if (driverController.getRightTriggerAxis() > 0.5) {
      // Right trigger to outtake
      intakeSubsystem.score();
    } else {
      // Disable otherwise
      intakeSubsystem.stop();
    }
  }

  @Override
  public void robotPeriodic() {
    // Run all subsystem periodic methods
    CommandScheduler.getInstance().run();
  }
}
