package frc.robot.subsystems.algae;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeEffectorIOSpark implements AlgaeEffectorIO {
  // two motors (pivot and intake)
  private final SparkMax pivotMotor;
  private final SparkMax intakeMotor;
  // TODO: schematics
  private static final int PIVOT_ID = 1;
  private static final int INTAKE_ID = 2;

  public AlgaeEffectorIOSpark() {
    pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    intakeMotor = new SparkMax(INTAKE_ID, MotorType.kBrushless);
  }

  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void setIntakeVolts(double volts) {
    intakeMotor.setVoltage(volts);
  }
}
