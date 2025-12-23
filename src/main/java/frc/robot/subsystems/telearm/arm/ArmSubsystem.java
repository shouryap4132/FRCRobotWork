// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telearm.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.telearm.arm.ArmSubsystemIO.ArmData;
import frc.robot.subsystems.telearm.arm.sim.ArmSim;
import edu.wpi.first.units.measure.Angle;


public class ArmSubsystem extends SubsystemBase {
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
  private ArmSubsystemIO armIO;
  private ArmData data = new ArmData();
  private ArmConstants.ArmStates state = ArmConstants.ArmStates.STOW;
  private ProfiledPIDController profile;
  private ArmFeedforward feedforward;

  public static double angleRad = 0.0;

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

  public static void setAngleRad(double angleInRad){
      angleRad = angleInRad;
  }

  public void stop(){
        armIO.setVoltage(0);
  }

  public void moveToGoal(){
        profile.setGoal(angleRad);
        double pidOutput = profile.calculate(data.positionRad, data.velocityRadsPerSecond);
        double ffOutput = feedforward.calculate(
            profile.getSetpoint().position,
            profile.getSetpoint().velocity);
        double gravtiyOutput = ArmConstants.kG * Math.cos(data.positionRad);
        setVoltage(pidOutput + ffOutput + gravityOutput);
  }

  @Override
public void periodic() {
    profile.setPID(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    // profile.setPID(0, 0, 0);
    ArmSubsystemIO.updateData(data);

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
