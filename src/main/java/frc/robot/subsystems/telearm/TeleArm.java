// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telearm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmSubsystemIO.ArmData;
import frc.robot.subsystems.arm.sim.ArmSim;
import edu.wpi.first.units.measure.Angle;

import frc.robot.subsystems.telearm.arm.ArmSubsystem;
import frc.robot.subsystems.telearm.arm.ArmConstants;
import frc.robot.subsystems.telearm.elevator.Elevator;
import frc.robot.subsystems.telearm.elevator.ElevatorConstants;
import frc.robot.utils.RectToPolar;




public class TeleArm extends SubsystemBase {
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

  public double getAngle(double xMeters, double yMeters){
      return RectToPolar.getAngle(xMeters, yMeters);
  }

  public double getLength(double xMeters, double yMeters){
      return RectToPolar.getLength(xMeters, yMeters);
  }

  

  public void moveGoal(double xMeters, double yMeters){
        double angle = getAngle(xMeters, yMeters);
        double length = getLength(xMeters, yMeters);
    
        ArmSubsystem arm = ArmSubsystem
        Elevator elevator = Robot.getRobotContainer().getTeleArm().getElevatorSubsystem();
    
        arm.setState(ArmConstants.ArmStates.fromAngle(angle));
        elevator.setState(ElevatorConstants.ElevatorStates.fromLength(length));

  }
  

  private ArmSubsystemIO armIO;
  private ArmData data = new ArmData();
  private ArmConstants.ArmStates state = ArmConstants.ArmStates.STOW;
  private ProfiledPIDController profile;
  private ArmFeedforward feedforward;

  public ArmSubsystem() {

    if (Robot.isSimulation()){
        armIO = new ArmSim();
    }

    profile = new ProfiledPIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocity,
            ArmConstants.kMaxAcceleration));
    feedforward = new ArmFeedforward(
        ArmConstants.kS,
        ArmConstants.kG,
        ArmConstants.kV,
        ArmConstants.kA);

    profile.reset(data.positionRad);
  }

//   private Angle getPitch(){
//       return Angle.ofBaseUnits(0, null)
//   }
  

  public ArmConstants.ArmStates getState(){
      return state;
  }

  public double getPosition(){
    return data.positionRad;
  }

  public void setVoltage(double volts){
    armIO.setVoltage(volts);
  }

  public void setState(ArmConstants.ArmStates newState){
      state = newState;
  }

  public void stop(){
        armIO.setVoltage(0);
  }

  private void moveToGoal(){
        profile.setGoal(state.setPointRad);
        double pidOutput = profile.calculate(data.positionRad, data.velocityRadsPerSecond);
        double ffOutput = feedforward.calculate(
            profile.getSetpoint().position,
            profile.getSetpoint().velocity);
        setVoltage(pidOutput + ffOutput);
  }

  @Override
public void periodic() {
    profile.setPID(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    // profile.setPID(0, 0, 0);
    armIO.updateData(data);

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
