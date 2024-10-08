package frc.robot.subsystems.Climb;

public interface IClimbIO {
    public static class ClimbIOInputs {

        public boolean m_leftLimitSwitchState;
        public boolean m_rightLimitSwitchState;

        public double m_leftVictorSPXOutputPercent = 0;
        public double m_rightVictorSPXOutputPercent = 0;
        
    }

    public static class ClimbIOOutputs {

    }

    public enum Side {
        kLeft,
        kRight
    }

    public enum Direction {
        kRaise,
        kLower
    }

    public ClimbIOInputs getInputs();

    public void setOutputs(ClimbIOOutputs outputs);

    public void MoveArm(Side side, Direction direction, double speed);

    public void StopArm(Side side);

    public void SetClutch(Side side, boolean engaged);
}
