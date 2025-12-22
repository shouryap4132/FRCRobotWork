package frc.robot.subsystems.telearm.elevator.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.telearm.elevator.ElevatorIO;
import frc.robot.subsystems.telearm.elevator.ElevatorConstants;
import frc.robot.utils.MiscConstants.SimConstants;



public class ElevatorSimulation implements ElevatorIO{
    
    private ElevatorSim elevatorSim;

    private double inputVolts = 0.0;
    private double previousVelocity = 0.0;

    

    public ElevatorSimulation() {
        elevatorSim = new ElevatorSim(
            DCMotor.getNEO(ElevatorConstants.numMotors),
            ElevatorConstants.gearing,
            ElevatorConstants.carriageMassKg,
            ElevatorConstants.sprocketRadiusMeters,
            ElevatorConstants.minHeightMeters,
            ElevatorConstants.maxHeightMeters,
            false, ElevatorConstants.startingHeightMeters, null
        );
    }

    @Override
    public void updateData(ElevatorData data) {
        elevatorSim.update(0.02);
        double velocity = elevatorSim.getVelocityRadPerSec();
		data.positionRad = elevatorSim.getAngleRads();
		data.velocityRadsPerSecond = velocity;
		data.accelerationRadsPerSecondSquared = ( velocity - previousVelocity ) / SimConstants.loopPeriodSec;
		data.motorCurrentAmps = elevatorSim.getCurrentDrawAmps();
		data.motorAppliedVolts = inputVolts;
		data.motorTempCelcius = 0;

        data.positionMeters = elevatorSim.getPositionMeters();
        data.velocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.currentAmps = elevatorSim.getCurrentDrawAmps();
        data.appliedVolts = inputVolts;
        data.tempCels = 0;

    }

    @Override

    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12, 12);
        armSim.setInputVoltage(inputVolts);
    }

}
