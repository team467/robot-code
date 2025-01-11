package frc.robot.subsystems.algae;

public interface AlgaeEffectorIO {
  class AlgaeEffectorIOInputs {
    // intake
    public double intakeVelocity;
    public double intakeVolts;
    public double intakeAmps;
    // pivot
    public double pivotVelocity;
    public double pivotVolts;
    public double pivotAmps;
    public double pivotPosition;
  }

  default void updateInputs(AlgaeEffectorIOInputs inputs) {} // TODO add AutoLogged

  default void setPivotVolts(double volts) {}

  default void setIntakeVolts(double volts) {}
}
