package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxController implements SpeedControllerEncoder {
    CANSparkMax spark;
    
    public SparkMaxController(int id, MotorType motorType) {
        spark = new CANSparkMax(id, motorType);
    }

    @Override
    public void set(double speed) {
        spark.set(speed);
    }

    @Override
    public double get() {
        return spark.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        spark.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return spark.getInverted();
    }

    @Override
    public void disable() {
        spark.disable();
    }

    @Override
    public void stopMotor() {
        spark.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        spark.pidWrite(output);
    }

    @Override
    public double getPosition() {
        return spark.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return spark.getEncoder().getVelocity();
    }
    
}
