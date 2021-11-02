package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonController implements SpeedControllerEncoder {
    WPI_TalonSRX talon;
    
    public TalonController(int id) {
        talon = new WPI_TalonSRX(id);
    }

    @Override
    public void set(double speed) {
        talon.set(speed);
    }

    @Override
    public double get() {
        return talon.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        talon.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return talon.getInverted();
    }

    @Override
    public void disable() {
        talon.disable();
    }

    @Override
    public void stopMotor() {
        talon.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        talon.pidWrite(output);
    }

    @Override
    public double getPosition() {
        return talon.getSelectedSensorPosition();
    }

    @Override
    public double getVelocity() {
        return talon.getSelectedSensorVelocity();
    }
    
}
