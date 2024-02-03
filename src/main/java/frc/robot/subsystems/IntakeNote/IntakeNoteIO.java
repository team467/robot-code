package frc.robot.subsystems.IntakeNote;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeNoteIO {
  @AutoLog
  class IntakeNoteIOInputs {
    // I/p for intake
    public boolean seesNote = false; // Need to communicate with Vision
    public boolean armSwitchClosed = false;

    // O/p for intake
    public BooleanSupplier hasNote = () -> false;

    // Motor IO
    public double appliedVolts = 0.0;
    public double motorVelocity = 0.0;
    public double motorCurrent = 0.0;
    public double motorPosition = 0.0;
  }

  default void updateInputs(IntakeNoteIOInputs intakeInputs) {}

  default void setSpeed(double speed) {} // [-1,1], -1 for release, 1 for intake
}
