package frc.robot.subsystems.IntakeNote;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeNoteIO {
  @AutoLog
  class IntakeNoteIOInputs {
    // I/p for intake
    public boolean seesNote = false; // Need to communicate with Vision
    public boolean armSwitchClosed = false;

    // O/p for intake

    // Motor IO
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeNoteIOInputs intakeInputs) {}

  default void setSpeed(double speed) {} // [-1,1], -1 for release, 1 for intake
}
