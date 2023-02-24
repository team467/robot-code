package frc.robot.subsystems.intakerelease;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class IntakeReleaseIOPhysical implements IntakeReleaseIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final DigitalInput cubeLimitSwitch;
  private final SparkMaxLimitSwitch coneLimitSwitch;

  public IntakeReleaseIOPhysical(int motorID, int cubeLimID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12);
    cubeLimitSwitch = new DigitalInput(cubeLimID);
    coneLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void setVelocity(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(IntakeReleaseIOInputs inputs, Wants wants) {
    inputs.motorPosition = encoder.getPosition();
    inputs.motorVelocity = encoder.getVelocity();
    inputs.motorAppliedVolts = motor.getBusVoltage();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.motorTemp = motor.getMotorTemperature();
    inputs.cubeLimitSwitch = cubeLimitSwitch.get();
    inputs.coneLimitSwitch = coneLimitSwitch.isPressed();
    inputs.wantsCone = wants == Wants.CONE ? true : false;
    inputs.wantsCube = wants == Wants.CUBE ? true : false;
  }
}
