package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//import org.littletonrobotics.junction.AutoLog;

public interface IShooterIO {
    
    //@AutoLog
    public static class ShooterIOInputs{

        public boolean m_noteDetectorState = false;

        public double m_talonFXVelocity = 0;
        public double m_talonFXState;

        public double m_victorSPXOutputPercent = 0;

        public Value m_elevationSolenoidState;

    }

    //@AutoLog
    public static class ShooterIOOutputs{ 

        public double m_talonFXSpeed = 0;
        public double m_victorSPXSpeed = 0;

        public boolean m_stopTalonFX = false;
        public boolean m_stopVictorSPX = false;

        public Value m_elevationSolenoidValue;

        public boolean m_playNoteDetectedLedPattern = false;
        public boolean m_startShootingNoteLedPattern = false;
        
    }

    public ShooterIOInputs getInputs();

    public void setOutputs(ShooterIOOutputs outputs);
}