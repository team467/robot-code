package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.led.Led2023;

public class ArmShelfCMD extends ArmPositionCMD {

  public ArmShelfCMD(Arm arm, Led2023 ledStrip) {

    super(arm, ArmPositionConstants.SHELF, ledStrip);
  }
}