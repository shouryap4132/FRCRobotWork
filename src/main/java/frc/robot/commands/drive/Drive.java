// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.driveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
public class Drive extends Command {

  private final driveSubsystem drive;

  private final double target;

  public Drive(driveSubsystem subsystem, double rpm) {
    drive = subsystem;
    target = rpm;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    drive.setTargetSpeed(target);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}