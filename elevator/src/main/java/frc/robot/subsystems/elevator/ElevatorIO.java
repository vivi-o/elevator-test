package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMetersPerSec = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] elevatorTempCelsius = new double[] {};
    }

    public void updateInputs(final ElevatorIOInputsAutoLogged Inputs);

    public void setTarget(final double meters);

    public void setVoltage(final double voltage);

    public default void stop() {
        setVoltage(0);
    }

    public void resetEncoder(final double position);

    //public default void resetEncoder(final double position) {

    public default  void resetEncoder()  {
        resetEncoder(0.0);
    }

}
