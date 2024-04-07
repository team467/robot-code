// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.AutoChooser.AutoQuestionResponse.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.characterization.FeedForwardCharacterization;
import frc.lib.characterization.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.lib.io.gyro3d.GyroIO;
import frc.lib.io.gyro3d.GyroPigeon2;
import frc.lib.io.vision.Vision;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.StolenJoystick;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMAX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOPhysical;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOPhysical;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.pixy2.Pixy2IO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOPhysical;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Subsystem subsystem;
  private Shooter shooter;
  private Indexer indexer;
  private Intake intake;
  private Drive drive;
  private Arm arm;
  private Vision vision;
  private Pixy2 pixy2;
  private Leds leds;
  private Climber climber;
  private boolean isRobotOriented = true; // Workaround, change if needed

  private Orchestrator orchestrator;
  private Autos autos;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final AutoChooser autoChooser = new AutoChooser("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate active subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023 -> {
          drive =
              new Drive(
                  new GyroPigeon2(Schematic.GYRO_ID),
                  new ModuleIOSparkMAX(0),
                  new ModuleIOSparkMAX(1),
                  new ModuleIOSparkMAX(2),
                  new ModuleIOSparkMAX(3));
        }
        case ROBOT_2024_COMP -> {
          Transform3d front =
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(6.74),
                      Units.inchesToMeters(-10.991),
                      Units.inchesToMeters(15.875)),
                  new Rotation3d(0, Units.degreesToRadians(-30), 0));
          Transform3d back =
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-11.89),
                      Units.inchesToMeters(0),
                      Units.inchesToMeters(15.5)),
                  new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));

          vision =
              new Vision(
                  List.of(
                          new VisionIOPhotonVision("front", front),
                          new VisionIOPhotonVision("back", back))
                      .toArray(new frc.lib.io.vision.VisionIO[0]));

          drive =
              new Drive(
                  new GyroPigeon2(Schematic.GYRO_ID),
                  new ModuleIOSparkMAX(0),
                  new ModuleIOSparkMAX(1),
                  new ModuleIOSparkMAX(2),
                  new ModuleIOSparkMAX(3));
          arm = new Arm(new ArmIOSparkMAX());
          indexer = new Indexer(new IndexerIOPhysical());
          intake = new Intake(new IntakeIOPhysical());
          shooter = new Shooter(new ShooterIOPhysical());
          leds = new Leds();
          // pixy2 = new Pixy2(new Pixy2IOPhysical());
          climber = new Climber(new ClimberIOSparkMax());
        }

        case ROBOT_SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
        }
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (pixy2 == null) {
      pixy2 = new Pixy2(new Pixy2IO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    orchestrator = new Orchestrator(drive, intake, indexer, shooter, pixy2, arm);

    // Rumble on intake
    new Trigger(() -> RobotState.getInstance().hasNote)
        .onTrue(
            Commands.runEnd(
                    () -> driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1),
                    () -> driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0))
                .withTimeout(1.2)
                .ignoringDisable(true)
                .withName("rumble"));

    // Configure the button bindings and auto options
    configureButtonBindings();
    configureAutoChoices();
  }

  private void configureAutoChoices() {
    autos = new Autos(drive, arm, orchestrator, autoChooser::getResponses);
    registerNamedCommands();

    // Set up auto routines

    autoChooser.addOption(
        "Drive Characterization",
        Commands.runOnce(() -> drive.setPose(new Pose2d()), drive)
            .andThen(
                new FeedForwardCharacterization(
                    drive,
                    true,
                    new FeedForwardCharacterizationData("drive"),
                    drive::runCharacterizationVolts,
                    drive::getCharacterizationVelocity))
            .andThen(this::configureButtonBindings));

    autoChooser.addOption(
        "Mobility",
        List.of(
            new AutoChooser.AutoQuestion(
                "Starting Position?",
                List.of(
                    AutoChooser.AutoQuestionResponse.RIGHT,
                    AutoChooser.AutoQuestionResponse.CENTER,
                    AutoChooser.AutoQuestionResponse.LEFT),
                CENTER),
            new AutoChooser.AutoQuestion("Score Preloaded?", List.of(YES, NO), YES),
            new AutoChooser.AutoQuestion("With Delay?", List.of(YES, NO), YES)),
        autos.mobilityOptions());

    autoChooser.addOption("Score One Note Only", autos.oneNoteAuto());

    autoChooser.addOption(
        "Score Two Notes",
        List.of(
            new AutoChooser.AutoQuestion(
                "Starting Position",
                List.of(
                    AutoChooser.AutoQuestionResponse.RIGHT,
                    AutoChooser.AutoQuestionResponse.CENTER,
                    AutoChooser.AutoQuestionResponse.LEFT),
                CENTER)),
        autos.noVisionTwoNoteAuto(
            () -> Autos.StartingPosition.valueOf(autoChooser.getResponses().get(0).toString())));

    autoChooser.addOption(
        "Score Three Notes",
        List.of(
            new AutoChooser.AutoQuestion(
                "Starting Position",
                List.of(
                    AutoChooser.AutoQuestionResponse.RIGHT,
                    AutoChooser.AutoQuestionResponse.CENTER,
                    AutoChooser.AutoQuestionResponse.LEFT),
                CENTER),
            new AutoChooser.AutoQuestion("3rd Note Position?", List.of(AMP, STAGE), AMP)
                .conditional(
                    () -> {
                      List<AutoChooser.AutoQuestionResponse> responses = autoChooser.getResponses();
                      return !responses.isEmpty() && responses.get(0).equals(CENTER);
                    })),
        autos.threeNoteAuto());
    autoChooser.addOption("Score Four Notes [CENTER]", autos.noVisionFourNoteAuto());
  }

  private void registerNamedCommands() {
    // Named commands for PathPlanner Auto
    NamedCommands.registerCommand(
        "intake",
        orchestrator
            .intakeBasic()
            .beforeStarting(orchestrator.stopFlywheel().withTimeout(0.2))
            .withTimeout(3));
    NamedCommands.registerCommand("spinFlywheel", orchestrator.spinUpFlywheel().withTimeout(0.6));
    NamedCommands.registerCommand("shoot", orchestrator.shootBasic());
    NamedCommands.registerCommand("oneNote", autos.oneNoteAuto());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    driverController
        .y()
        .onTrue(
            Commands.runOnce(() -> isRobotOriented = !isRobotOriented)
                .withName("robotOrientedToggle"));
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> isRobotOriented // TODO: add toggle
            ));
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true)
                .withName("resetGyro"));
    driverController
        .pov(-1)
        .whileFalse(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));
    // stop when doing nothing
    intake.setDefaultCommand(intake.stop());
    indexer.setDefaultCommand(indexer.setPercent(0));
    shooter.setDefaultCommand(shooter.manualShoot(0));
    arm.setDefaultCommand(arm.hold());
    climber.setDefaultCommand(climber.raiseOrLower(0));

    // operator controller
    // Hold A: Spin up shooter
    operatorController.a().whileTrue(orchestrator.spinUpFlywheel());
    // Hold B: Expel intake
    operatorController.b().whileTrue(orchestrator.expelIntakeIndex());
    // Click X: Move arm to stow position
    operatorController.x().onTrue(arm.toSetpoint(ArmConstants.STOW));
    // Hold Y: Expel the shooter
    operatorController.y().whileTrue(shooter.manualShoot(-1));
    // Hold RB: Duck the arm to fit under stage
    operatorController.rightBumper().whileTrue(orchestrator.duck());
    operatorController.rightBumper().onFalse(orchestrator.unDuck());

    // Back button (toggle switch): unlock/lock climber ratchet
    operatorController.back().whileTrue(climber.setRatchet(false));
    operatorController.back().whileFalse(climber.setRatchet(true));
    // Click LB: turn to speaker and align the arm to speaker
    // operatorController.leftBumper().onTrue(orchestrator.fullAlignSpeaker(() -> drive.getPose()));

    // operator d pad
    // Hold Up: Move arm up
    operatorController.pov(0).whileTrue(arm.runPercent(0.2));
    // Hold Down: Move arm down
    operatorController.pov(180).whileTrue(arm.runPercent(-0.2));
    // Hold Right: Move climber up
    operatorController
        .pov(90)
        .whileTrue(
            climber
                .raiseOrLower(ClimberConstants.CLIMBER_BACKWARD_PERCENT)
                .withTimeout(ClimberConstants.BACKUP_TIME)
                .andThen(climber.raiseOrLower(ClimberConstants.CLIMBER_FORWARD_PERCENT)));
    // Hold Left: Move climber down
    operatorController
        .pov(270)
        .whileTrue(climber.raiseOrLower(ClimberConstants.CLIMBER_BACKWARD_PERCENT));

    driverController
        .rightBumper()
        .toggleOnTrue(
            Commands.parallel(
                    new StolenJoystick(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> drive.getPose(),
                        FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d(),
                        () -> true),
                    orchestrator.alignArmSpeaker(() -> drive.getPose()))
                .withName("stolenJoystickToggle"));
    // Click Left Bumper: Move arm to amp position or home position
    driverController
        .leftBumper()
        .onTrue(
            Commands.sequence(
                    orchestrator
                        .armToHome()
                        .onlyIf(() -> arm.getAngle() > Units.degreesToRadians(65)),
                    orchestrator
                        .alignArmAmp()
                        .onlyIf(() -> arm.getAngle() < Units.degreesToRadians(65)))
                .withName("armPositionChanger"));
    // Click left Trigger: Intake (until clicked again or has a note)
    driverController.leftTrigger(0.15).toggleOnTrue(orchestrator.intakeBasic());
    // Click right Trigger: Run indexer
    driverController.rightTrigger(0.15).onTrue(orchestrator.indexBasic());
    // Click A: X lock drive train
    driverController.a().onTrue(Commands.runOnce(() -> drive.stopWithX()).withName("xlock"));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
