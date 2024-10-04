package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface IShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs{
        
    }

    @AutoLog
    public static class ShooterIOOutputs{ 
        
    }

    public ShooterIOInputs getInputs();

    public void setOutputs(ShooterIOOutputs outputs);
}