package frc.robot.subsystems.effector;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorIO {
  @AutoLog
  class EffectorIOInputs {
    public double effectorVelocityRadPerSec;
    public double effectorAppliedVolts;
    public double effectorCurrentAmps;
    public boolean effectorLimitSwitchPressed;
  }

  default void updateInputs(EffectorIOInputs inputs) {}

  default void setEffectorVoltage(double volts) {}

  default void setEffectorVelocity(double velocity) {}
}
