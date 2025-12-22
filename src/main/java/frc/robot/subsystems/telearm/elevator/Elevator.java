// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telearm.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.telearm.elevator.ElevatorIO.ElevatorData;
import frc.robot.subsystems.telearm.elevator.sim.ElevatorSimulation;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;


public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  private ElevatorIO elevatorIO;
  private ElevatorData data = new ElevatorData();
  private ElevatorConstants.ElevatorStates state = ElevatorConstants.ElevatorStates.STOW;
  private ProfiledPIDController profile;
  private ArmFeedforward feedforward;

  public Elevator() {

    if (Robot.isSimulation()){
        elevatorIO = new ElevatorSimulation();
    }

    profile = new ProfiledPIDController(
        ElevatorConstants.ElevatorControl.kP,
        ElevatorConstants.ElevatorControl.kI,
        ElevatorConstants.ElevatorControl.kD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.ElevatorControl.maxVelocity,
            ElevatorConstants.ElevatorControl.maxAcceleration));
    feedforward = new ArmFeedforward(
        ElevatorConstants.ElevatorControl.kS,
        ElevatorConstants.ElevatorControl.kG,
        ElevatorConstants.ElevatorControl.kV,
        ElevatorConstants.ElevatorControl.kA);

    profile.reset(data.positionMeters);
  }

//   private Angle getPitch(){
//       return Angle.ofBaseUnits(0, null)
//   }
  

  public ElevatorConstants.ElevatorStates getState(){
      return state;
  }

  public double getPosition(){
    return data.positionMeters;
  }

  public void setVoltage(double volts){
    ElevatorIO.setVoltage(volts);
  }

  public void setState(ElevatorConstants.ElevatorStates newState){
      state = newState;
  }

  public void stop(){
        ElevatorIO.setVoltage(0);
  }

  private void moveToGoal(){
        profile.setGoal(state.heightMeters);
        double pidOutput = profile.calculate(data.positionMeters, data.velocityMetersPerSecond);
        double ffOutput = feedforward.calculate(
            profile.getSetpoint().position,
            profile.getSetpoint().velocity);
        setVoltage(pidOutput + ffOutput);
  }

  @Override
public void periodic() {
    profile.setPID(ElevatorConstants.ElevatorControl.kP, ElevatorConstants.ElevatorControl.kI, ElevatorConstants.ElevatorControl.kD);
    // profile.setPID(0, 0, 0);
    ElevatorIO.updateData(data);

    moveToGoal();

    // logData();

}
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
