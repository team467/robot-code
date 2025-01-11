package frc.robot.subsystems.reflector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Reflector extends SubsystemBase {
  private DigitalInput photoelectricSensor;

  public Reflector() {
    photoelectricSensor =
        new DigitalInput(9); // notsure, also change number t actual dio port used.*/
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Reflector/CoralDetected", photoelectricSensor.get());
  }
}
