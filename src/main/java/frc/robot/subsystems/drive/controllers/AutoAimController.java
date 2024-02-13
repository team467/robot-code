package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.TunableNumber;
import frc.robot.RobotOdometry;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAimController {
  private static final TunableNumber headingkP = new TunableNumber("AutoAim/kP", 1.0);
  private static final TunableNumber headingkD = new TunableNumber("AutoAim/kD", 0.0);
  private static final TunableNumber tolerance = new TunableNumber("AutoAim/ToleranceDegrees", 4.0);

  private PIDController headingController;

  public AutoAimController() {
    headingController = new PIDController(0, 0, 0, 0.02);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update() {
    headingController.setPID(headingkP.get(), 0, headingkD.get());
    headingController.setTolerance(Units.degreesToRadians(tolerance.get()));

    var aimingParams = RobotOdometry.getInstance().getAimingParameters();
    double output =
        headingController.calculate(
            RobotOdometry.getInstance().getLatestPose().getRotation().getRadians(),
            aimingParams.driveHeading().getRadians());

    Logger.recordOutput("AutoAim/HeadingError", headingController.getPositionError());
    return output;
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "AutoAim/AtSetpoint")
  public boolean atSetpoint() {
    return headingController.atSetpoint();
  }
}
