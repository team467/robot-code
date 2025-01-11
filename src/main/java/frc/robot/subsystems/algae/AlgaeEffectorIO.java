package frc.robot.subsystems.algae;

public interface AlgaeEffectorIO {
  class AlgaeEffectorIOInputs {
    // intake
    public double intakeVelocity;
    public double intakeAmps;
    public double intakeVolts;
    // pivot
    public double pivotVelocity;
    public double pivotAmps;
    public double pivotVolts;
    public double pivotPosition;
  }

  default void updateInputs(AlgaeEffectorIOInputs inputs) {}
  // add in the auto logged parts to AlgaeEffectorIOInputs
  default void setIntakeVolts(double volts) {}

  default void setPivotVolts(double volts) {}
}
