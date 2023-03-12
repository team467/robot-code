package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmManualDownCMD extends CommandBase {
  private Arm arm;

  public ArmManualDownCMD(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.manualRotate(-7);
  }

  @Override
  public void end(boolean interrupted) {
    arm.hold();
  }
}
