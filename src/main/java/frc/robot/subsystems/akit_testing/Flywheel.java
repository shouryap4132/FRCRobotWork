package frc.robot.subsystems.akit_testing;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;

public class Flywheel extends SubsystemBase{
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.0);


    private double velocity = 0.0;
    private double targetVelocity = 100.0;
    

    @Override
    
    public void periodic() {
    // This method will be called once per scheduler run
    double currentkP = kP.get();

    double error = targetVelocity - velocity;
    double output = currentkP * error;

    velocity += output * 0.02;

    Logger.recordOutput("Flywheel/Velocity", velocity);
    Logger.recordOutput("Flywheel/Error", error);
    }
}
