package frc.robot.subsystems.effector;

import static frc.lib.utils.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.effector.EffectorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

public class EffectorIOSparkMax implements EffectorIO {
  private final SparkMax effector;

  private final RelativeEncoder effectorEncoder;

  private final SparkLimitSwitch effectorLimitSwitch;
  double rotsToRads = Units.rotationsToRadians(1);

  public EffectorIOSparkMax() {
    effector = new SparkMax(1, MotorType.kBrushless);
    effectorLimitSwitch = effector.getReverseLimitSwitch();
    effectorEncoder = effector.getEncoder();
    var effectorConfig = new SparkFlexConfig();
    effectorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(effectorMotorCurrentLimit)
        .voltageCompensation(12.0);
    effectorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    effectorConfig
        .encoder
        .positionConversionFactor(effectorEncoderPositionFactor)
        .velocityConversionFactor(effectorEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    tryUntilOk(
        effector,
        5,
        () ->
            effector.configure(
                effectorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    tryUntilOk(effector, 5, () -> effectorEncoder.setPosition(0.0));
  }

  public void updateInputs(EffectorIOInputs inputs) {
    inputs.effectorVelocityRadPerSec = effectorEncoder.getVelocity();
    inputs.effectorAppliedVolts = effector.getBusVoltage() * effector.getAppliedOutput();
    inputs.effectorCurrentAmps = effector.getOutputCurrent();
    inputs.effectorLimitSwitchPressed = effectorLimitSwitch.isPressed();
  }

  public void setEffectorVoltage(double volts) {
    effector.setVoltage(volts);
  }

  public void setEffectorVelocity(double velocity) {
    effector.set(velocity);
  }
}
