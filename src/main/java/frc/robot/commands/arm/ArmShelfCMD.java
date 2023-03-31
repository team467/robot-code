package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakerelease.IntakeAndRaise;
import frc.robot.commands.intakerelease.IntakeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmShelfCMD extends SequentialCommandGroup {

  public ArmShelfCMD(Arm arm, IntakeRelease intakeRelease) {
    addCommands(
        Commands.parallel(
            new ArmPositionCMD(
                arm,
                () ->
                    intakeRelease.wantsCone()
                        ? ArmPositionConstants.SHELF_CONE
                        : ArmPositionConstants.SHELF_CUBE),
            new IntakeCMD(intakeRelease)),
        new IntakeAndRaise(arm, intakeRelease));
  }
}
