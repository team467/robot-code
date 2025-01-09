package frc.robot.subsystems.effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Effector extends SubsystemBase {

  /** Creates a new effector. */
  private final EffectorIO io;

  private final RobotState robotState = RobotState.getInstance();
  private final EffectorIOInputsAutoLogged inputs = new EffectorIOInputsAutoLogged();

  public Effector(EffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Effector", inputs);
  }

  /**
   * @param percent the percent velocity that the effector should be set to
   * @return A command that sets the effectors velocity to that of the percent velocity inputted
   */
  public Command setEffectorVelocity(double percent) {
    return Commands.run(
        () -> {
          io.setEffectorVelocity(percent);
        },
        this);
  }
  /**
   * @param volts the volts that the effector should be set to
   * @return A command that sets the effector voltage to that of the inputted volts
   */
  public Command setEffectorVolts(double volts) {
    return Commands.run(
        () -> {
          io.setEffectorVoltage(volts);
        },
        this);
  }
}
