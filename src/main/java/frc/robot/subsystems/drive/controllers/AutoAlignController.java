package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
    @Getter(onMethod_ = @AutoLogOutput(key = "AutoAlign/goalPose"))
    private Pose2d goalPose = null;

    // Controllers for translation and rotation
    private final ProfiledPIDController linearController;
    private final ProfiledPIDController thetaController;

    // Store previous velocities for acceleration limiting
    private Translation2d prevLinearVelocity;

    public AutoAlignController(Pose2d goalPose) {
        this.goalPose = goalPose;
        // Set up both controllers
        linearController =
                new ProfiledPIDController(
                        linearkP.get(),
                        0,
                        linearkD.get(),
                        new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
        linearController.setTolerance(linearTolerance.get());
        thetaController =
                new ProfiledPIDController(
                        thetakP.get(),
                        0,
                        thetakD.get(),
                        new TrapezoidProfile.Constraints(
                                maxAngularVelocity.get(), maxAngularAcceleration.get()));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(thetaTolerance.get());

        // Reset measurements and velocities
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        Twist2d fieldVelocity = RobotState.getInstance().fieldVelocity();
        // Linear controller will control to 0 so distance is the measurement
        Rotation2d rotationToGoal =
                goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        double velocity =
                -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                        .rotateBy(rotationToGoal.unaryMinus())
                        .getX();
        linearController.reset(
                currentPose.getTranslation().getDistance(goalPose.getTranslation()), velocity);
        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);

        // Set goal positions
        linearController.setGoal(0.0);
        thetaController.setGoal(goalPose.getRotation().getRadians());

        // Store linear velocity for acceleration limiting
        prevLinearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy);

        // Log goal pose
        Logger.recordOutput("AutoAlign/GoalPose", goalPose);
    }

    public ChassisSpeeds update() {
        // Update PID Controllers
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
                linearkP,
                linearkD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
                thetakP,
                thetakD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () ->
                        linearController.setConstraints(
                                new TrapezoidProfile.Constraints(
                                        maxLinearVelocity.get(), maxLinearAcceleration.get())),
                maxLinearVelocity,
                maxLinearAcceleration);

        // Control to setpoint
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

        // Calculate feedback velocities (based on position error).
        double linearVelocityScalar =
                linearController.calculate(
                                currentPose.getTranslation().getDistance(goalPose.getTranslation()))
                        + linearController.getSetpoint().velocity;
        Rotation2d rotationToGoal =
                goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        Translation2d desiredLinearVelocity = new Translation2d(-linearVelocityScalar, rotationToGoal);

        double angularVelocity =
                thetaController.calculate(
                                currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians())
                        + thetaController.getSetpoint().velocity;

        ChassisSpeeds fieldRelativeSpeeds =
                new ChassisSpeeds(
                        desiredLinearVelocity.getX(), desiredLinearVelocity.getY(), angularVelocity);
        Logger.recordOutput("AutoAlign/FieldRelativeSpeeds", fieldRelativeSpeeds);
        Logger.recordOutput("AutoAlign/LinearError", linearController.getPositionError());
        Logger.recordOutput("AutoAlign/RotationError", thetaController.getPositionError());
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());
    }

    @AutoLogOutput(key = "AutoAlign/AtGoal")
    public boolean atGoal() {
        return linearController.atGoal() && thetaController.atGoal();
    }
}
