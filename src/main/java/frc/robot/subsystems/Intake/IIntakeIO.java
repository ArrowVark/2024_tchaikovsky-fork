package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

public interface IIntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        public DigitalInput m_topLimitSwitch;
        public DigitalInput m_bottomLimitSwitch;

        public double m_angleLeftPosition;
        public double m_angleRightPosition;

    }

    @AutoLog
    public static class IntakeIOOutputs { 
        
        public boolean m_angleToggledIn;
        public double m_angleStartPoint;

        public double m_rollersSpeed;
        public double m_angleLeftSpeed;
        public double m_angleRightSpeed;

        public boolean m_stopRollers = false;
        public boolean m_stopAngleLeft = false;
        public boolean m_stopAngleRight = false;

    }

    public IntakeIOInputs getInputs();

    public void setOutputs(IntakeIOOutputs outputs);
}
