package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();
  // private final Relay climberLock = new Relay(); //To do add Soloenoid

  public Climber(ClimberIO climberIO) {
    super();

    this.climberIO = climberIO;

    climberIO.updateInput(climberIOInputs);
  }

  public Command raise(double percentOutput) {
    return Commands.run(
        () -> {
          // climberIO.setLeftMotorOutputPercent(percentOutput);
          // climberIO.setRightMotorOutputPercent(percentOutput);
        },
        this);
  }

  public Command lower(double percentOutput) {
    return Commands.run(
        () -> {
          // climberIO.setLeftMotorOutputPercent(percentOutput);
          // climberIO.setRightMotorOutputPercent(percentOutput);
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          climberIO.setLeftMotorOutputPercent(0);
          climberIO.setRightMotorOutputPercent(0);
        },
        this);
  }

  public Command disable() {
    return Commands.run(
        () -> {
          climberIO.setLeftMotorOutputPercent(0);
          climberIO.setRightMotorOutputPercent(0);
        },
        this);
  }
}
