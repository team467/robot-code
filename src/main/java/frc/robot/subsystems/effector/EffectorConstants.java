package frc.robot.subsystems.effector;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class EffectorConstants {
  public static final TunableNumber EFFECTOR_SPEED_OUT;
  public static final TunableNumber EFFECTOR_SPEED_IN;
  public static final TunableNumber EFFECTOR_HOLD_PERCENT;
  public static final int effectorMotorCurrentLimit;
  public static final int effectorEncoderPositionFactor;
  public static final int effectorEncoderVelocityFactor;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT -> {
        EFFECTOR_SPEED_OUT = new TunableNumber("Effector/SpeedOut", 0);
        EFFECTOR_SPEED_IN = new TunableNumber("Effector/SpeedIn", 0);
        EFFECTOR_HOLD_PERCENT = new TunableNumber("Effector/HoldPercent", 0);
        effectorMotorCurrentLimit = 0;
        effectorEncoderPositionFactor = 0;
        effectorEncoderVelocityFactor = 0;
      }
      case ROBOT_2025_COMP -> {
        EFFECTOR_SPEED_OUT = new TunableNumber("Effector/SpeedOut", 0.5);
        EFFECTOR_SPEED_IN = new TunableNumber("Effector/SpeedIn", -0.5);
        EFFECTOR_HOLD_PERCENT = new TunableNumber("Effector/HoldPercent", 0.1);
        effectorMotorCurrentLimit = 35;
        effectorEncoderPositionFactor = 1;
        effectorEncoderVelocityFactor = 1;
      }
      default -> {
        EFFECTOR_SPEED_OUT = new TunableNumber("Effector/SpeedOut", 0);
        EFFECTOR_SPEED_IN = new TunableNumber("Effector/SpeedIn", 0);
        EFFECTOR_HOLD_PERCENT = new TunableNumber("Effector/HoldPercent", 0);
        effectorMotorCurrentLimit = 0;
        effectorEncoderPositionFactor = 0;
        effectorEncoderVelocityFactor = 0;
      }
    }
  }
}
