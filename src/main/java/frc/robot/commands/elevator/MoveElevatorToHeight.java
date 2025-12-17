package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class MoveElevatorToHeight extends Command {

  private final ElevatorSubsystem elevator;
  private final double targetHeightMeters;

  public MoveElevatorToHeight(
      ElevatorSubsystem elevator, double targetHeightMeters) {
    this.elevator = elevator;
    this.targetHeightMeters = targetHeightMeters;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setGoal(targetHeightMeters);
    System.out.println("Initialized");
  }

  @Override
  public boolean isFinished() {
    return elevator.atGoal();
  }

  @Override
  public void execute() {
    // elevator.setGoal(20);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
