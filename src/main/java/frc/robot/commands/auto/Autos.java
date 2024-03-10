package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.Orchestrator;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final Orchestrator orchestrator;

  @AutoLogOutput(key = "Autos/Notes/0")
  private Translation2d noteTranslation;

  @AutoLogOutput(key = "Autos/Notes/1")
  private Translation2d secondNoteTranslation;

  @AutoLogOutput(key = "Autos/Notes/2")
  private Translation2d thirdNoteTranslation;

  public Autos(Drive drive, Orchestrator orchestrator) {
    this.drive = drive;
    this.orchestrator = orchestrator;
  }

  public enum StartingPosition {
    LEFT(
        new Pose2d(
            AllianceFlipUtil.apply(
                    AllianceFlipUtil.shouldFlip()
                        ? FieldConstants.Subwoofer.sourceFaceCorner
                        : FieldConstants.Subwoofer.ampFaceCorner)
                .getTranslation(),
            AllianceFlipUtil.apply(
                    AllianceFlipUtil.shouldFlip()
                        ? FieldConstants.Subwoofer.sourceFaceCorner
                        : FieldConstants.Subwoofer.ampFaceCorner)
                .getRotation()
                .plus(Rotation2d.fromDegrees(180)))),
    CENTER(
        new Pose2d(
            AllianceFlipUtil.apply(FieldConstants.Subwoofer.centerFace.getTranslation()),
            AllianceFlipUtil.apply(
                FieldConstants.Subwoofer.centerFace
                    .getRotation()
                    .plus(Rotation2d.fromRadians(Units.degreesToRadians(180)))))),
    RIGHT(
        new Pose2d(
            AllianceFlipUtil.apply(
                    AllianceFlipUtil.shouldFlip()
                        ? FieldConstants.Subwoofer.ampFaceCorner
                        : FieldConstants.Subwoofer.sourceFaceCorner)
                .getTranslation(),
            AllianceFlipUtil.apply(
                    AllianceFlipUtil.shouldFlip()
                        ? FieldConstants.Subwoofer.ampFaceCorner
                        : FieldConstants.Subwoofer.sourceFaceCorner)
                .getRotation()
                .plus(Rotation2d.fromDegrees(180))));
    private final Pose2d startingPosition;

    StartingPosition(Pose2d startingPosition) {
      this.startingPosition = startingPosition;
      Logger.recordOutput("Autos/StartingPositions/" + this, startingPosition);
    }

    public Pose2d getStartingPosition() {
      return startingPosition;
    }
  }

  private Command setNotePositions(Supplier<StartingPosition> position) {
    return Commands.runOnce(
        () -> {
          switch (position.get()) {
            case RIGHT -> {
              this.noteTranslation = getNotePositions(0, false);
              this.secondNoteTranslation = getNotePositions(1, false);
              this.thirdNoteTranslation = getNotePositions(2, true);
            }
            case CENTER -> {
              this.noteTranslation = getNotePositions(1, false);
              this.secondNoteTranslation = getNotePositions(2, false);
              this.thirdNoteTranslation = getNotePositions(0, true);
            }
            case LEFT -> {
              this.noteTranslation = getNotePositions(2, false);
              this.secondNoteTranslation = getNotePositions(1, false);
              this.thirdNoteTranslation = getNotePositions(0, true);
            }
          }
        });
  }

  private Translation2d getNotePositions(int index, boolean centerNotes) {
    return AllianceFlipUtil.apply(
        centerNotes
            ? FieldConstants.StagingLocations.centerlineTranslations[
                AllianceFlipUtil.shouldFlip() ? 4 - index : index]
            : FieldConstants.StagingLocations.spikeTranslations[
                AllianceFlipUtil.shouldFlip() ? 2 - index : index]);
  }

  public Command mobilityAuto(StartingPosition position) {
    return Commands.runOnce(() -> drive.setPose(position.getStartingPosition()))
            .andThen(
                    Commands.either(
                            Commands.select(
                                    Map.of(
                                            StartingPosition.RIGHT,
                                            new StraightDriveToPose(Units.feetToMeters(-6.75), 0, 0, drive)
                                                    .withTimeout(5),
                                            StartingPosition.CENTER,
                                            new StraightDriveToPose(Units.feetToMeters(-6.75), 0, 0, drive)
                                                    .withTimeout(5),
                                            StartingPosition.LEFT,
                                            Commands.run(
                                                            () ->
                                                                    drive.runVelocity(
                                                                            new ChassisSpeeds(Units.feetToMeters(9), 0, 0)))
                                                    .withTimeout(1)
                                                    .andThen(
                                                            new StraightDriveToPose(Units.feetToMeters(-6.75), 0, 0, drive)
                                                                    .withTimeout(5))),
                                    () -> position),
                            Commands.select(
                                    Map.of(
                                            StartingPosition.LEFT,
                                            new StraightDriveToPose(Units.feetToMeters(6.75), 0, 0, drive)
                                                    .withTimeout(5),
                                            StartingPosition.CENTER,
                                            new StraightDriveToPose(Units.feetToMeters(-6.75), 0, 0, drive)
                                                    .withTimeout(5),
                                            StartingPosition.RIGHT,
                                            Commands.run(
                                                            () ->
                                                                    drive.runVelocity(
                                                                            new ChassisSpeeds(Units.feetToMeters(9), 0, 0)))
                                                    .withTimeout(1)
                                                    .andThen(
                                                            new StraightDriveToPose(Units.feetToMeters(6.75), 0, 0, drive)
                                                                    .withTimeout(5))),
                                    () -> position),
                            AllianceFlipUtil::shouldFlip));
  }

  public Command oneNoteAuto() {
    return orchestrator.shootBasic();
  }

  public Command scoreOneNoteMobility(StartingPosition position) {

    return Commands.sequence(oneNoteAuto().andThen(mobilityAuto(position)));
  }

  public Command twoNoteAuto(StartingPosition position) {
    return Commands.runOnce(() -> drive.setPose(position.getStartingPosition()))
        .andThen(setNotePositions(() -> position))
        .andThen(
            oneNoteAuto()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> noteTranslation).withTimeout(5),
                        orchestrator.intakeBasic()))
                .andThen(shoot()));
  }

  public Command threeNoteAuto(StartingPosition position) {
    return setNotePositions(() -> position)
        .andThen(
            oneNoteAuto()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> noteTranslation),
                        orchestrator.intakeBasic()))
                .andThen(shoot())
                .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(() -> secondNoteTranslation),
                            orchestrator.intakeBasic())
                        .andThen(shoot())));
  }

  private Command shoot() {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)))
        .onlyWhile(
            () ->
                drive
                        .getPose()
                        .getTranslation()
                        .getDistance(
                            AllianceFlipUtil.apply(
                                FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
                    > Units.inchesToMeters(30))
        .andThen(orchestrator.fullAlignShootSpeaker());
  }

  public Command fourNoteAuto(StartingPosition position) {
    return setNotePositions(() -> position)
        .andThen(
            oneNoteAuto()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> noteTranslation),
                        orchestrator.intakeBasic()))
                .andThen(shoot())
                .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(() -> secondNoteTranslation),
                            orchestrator.intakeBasic())
                        .andThen(shoot()))
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> thirdNoteTranslation),
                        orchestrator.intakeBasic()))
                .andThen(shoot()));
  }
}
