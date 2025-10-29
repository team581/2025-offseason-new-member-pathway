package frc.robot.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S2StateValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final TalonFX intakeMotor;
  private final CANdi intakeCandi;

  public IntakeSubsystem(TalonFX intakeMotor, CANdi intakeCandi) {
    this.intakeMotor = intakeMotor;
    this.intakeCandi = intakeCandi;
  }

  /** Runs the intake until a coral is detected. */
  public void intake() {
  }

  /** Runs the intake in reverse to score a coral. */
  public void score() {

  }

  /** Stops the intake. */
  public void stop() {

  }

  /** Returns whether the intake is holding a coral. */
  private boolean intakeHasCoral() {
    return false;
  }
}
