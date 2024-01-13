// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Affector;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
  /** Creates a new StartFlywheel. */
  private final IntakeNote intakeNote;

  public Intake(IntakeNote intakeNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeNote = intakeNote;
    addRequirements(intakeNote);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeNote.startIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
