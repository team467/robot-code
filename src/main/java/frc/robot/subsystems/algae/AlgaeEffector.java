package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEffector extends SubsystemBase {
  private final AlgaeEffectorIO io;
  private PIDController pivotFeedback =
      new PIDController(AlgaeEffefectorConstants.PIVOT_KP, 0, AlgaeEffefectorConstants.PIVOT_KD);
  // TODO: commands
  public AlgaeEffector(AlgaeEffectorIO io) {
    this.io = io;
  }
}
