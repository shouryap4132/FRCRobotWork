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
import frc.robot.subsystems.telearm.arm.ArmSubsystemIO.ArmData;
import frc.robot.subsystems.arm.sim.ArmSim;
import edu.wpi.first.units.measure.Angle;

import frc.robot.subsystems.telearm.arm.ArmSubsystem;
import frc.robot.subsystems.telearm.arm.ArmConstants;
import frc.robot.subsystems.telearm.elevator.Elevator;
import frc.robot.subsystems.telearm.elevator.ElevatorConstants;
import frc.robot.utils.RectToPolar;




public class TeleArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static double angle = 0.0;
  public static double length = 0.0;
  

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

  public void setLength(double lengthMeters){
      length = lengthMeters;
  }

  public void setAngleRad(double angleRad){
      angle = angleRad;
  }

  public void setGoal(double xMeters, double yMeters){
      angle = getAngle(xMeters, yMeters);
      length = getLength(xMeters, yMeters);
  }

  

  public void moveGoal(){
        // double angle = getAngle(xMeters, yMeters);
        // double length = getLength(xMeters, yMeters);
    
        ArmSubsystem.setAngleRad(angle);

        Elevator.setHeight(length);   
         
        
  }
    
  @Override
public void periodic() {

    moveGoal();
    // This method will be called once per scheduler run
    
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
