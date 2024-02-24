package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double motorPercentOutput = 0.0;
    public double motorVoltage = 0.0;
    public double currentAmpsLeft = 0.0;
    public double appliedVoltsLeft = 0.0;
    public double ClimberLeftPosition = 0.0;
    public double currentAmpsRight = 0.0;
    public double appliedVoltsRight = 0.0;
    public double ClimberRightPosition = 0.0;
    public boolean ratchetLocked = false;
  }

  default void updateInput(ClimberIOInputs inputs) {}

  default void setMotorsOutputPercent(double percentOutput) {}

  default void setLeftMotorVolts(double volts) {}

  default void setRightMotorVolts(double volts) {}

  default void setRatchetLocked(double locked) {}
}
