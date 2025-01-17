package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Schematic;

public class ModuleIOSparkMAX implements ModuleIO {
  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;

  private final SparkMax turnMotor;
  private final RelativeEncoder turnEncoder;

  private final CANcoder turnEncoderAbsolute;
  private final StatusSignal<Angle> turnAbsolutePosition;

  private int resetCount = 0;
  private final int index;

  public ModuleIOSparkMAX(int index) {
    int driveMotorId;
    int turnMotorId;
    int turnAbsEncoderId;
    switch (index) {
      case 0 -> {
        driveMotorId = Schematic.FRONT_LEFT_DRIVE_ID;
        turnMotorId = Schematic.FRONT_LEFT_STEERING_ID;
        turnAbsEncoderId = Schematic.FRONT_LEFT_CANCODER_ID;
      }
      case 1 -> {
        driveMotorId = Schematic.FRONT_RIGHT_DRIVE_ID;
        turnMotorId = Schematic.FRONT_RIGHT_STEERING_ID;
        turnAbsEncoderId = Schematic.FRONT_RIGHT_CANCODER_ID;
      }
      case 2 -> {
        driveMotorId = Schematic.REAR_LEFT_DRIVE_ID;
        turnMotorId = Schematic.REAR_LEFT_STEERING_ID;
        turnAbsEncoderId = Schematic.REAR_LEFT_CANCODER_ID;
      }
      case 3 -> {
        driveMotorId = Schematic.REAR_RIGHT_DRIVE_ID;
        turnMotorId = Schematic.REAR_RIGHT_STEERING_ID;
        turnAbsEncoderId = Schematic.REAR_RIGHT_CANCODER_ID;
      }
      default -> throw new IllegalArgumentException("Drive: Illegal index attempted " + index);
    }
    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    turnEncoderAbsolute = new CANcoder(turnAbsEncoderId);
    turnAbsolutePosition = turnEncoderAbsolute.getAbsolutePosition();

    double rotsToMeters =
        Units.rotationsToRadians(1)
            * (DriveConstants.WHEEL_DIAMETER / 2)
            * DriveConstants.DRIVE_GEAR_RATIO.getRotationsPerInput();
    double rotsToRads =
        Units.rotationsToRadians(1) * DriveConstants.TURN_GEAR_RATIO.getRotationsPerInput();

    driveMotor.configure(
        new SparkMaxConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .smartCurrentLimit(50)
            .apply(
                new AbsoluteEncoderConfig()
                    .positionConversionFactor(rotsToMeters / 60)
                    .velocityConversionFactor(rotsToRads / 60)),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    turnMotor.configure(
        new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .apply(
                new AbsoluteEncoderConfig()
                    .positionConversionFactor(rotsToMeters / 60)
                    .velocityConversionFactor(rotsToRads / 60)),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();

    // Reset the turn encoder sometimes when not moving
    if (turnEncoder.getVelocity() < Units.degreesToRadians(0.5)) {
      if (++resetCount >= 500) {
        resetCount = 0;
        turnEncoder.setPosition(
            MathUtil.angleModulus(
                Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                    .minus(DriveConstants.ABSOLUTE_ANGLE_OFFSET[index])
                    .getRadians()));
      }
    } else {
      resetCount = 0;
    }
    inputs.turnPosition = new Rotation2d(turnEncoder.getPosition());

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(DriveConstants.ABSOLUTE_ANGLE_OFFSET[index]);
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnMotor.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }
}
