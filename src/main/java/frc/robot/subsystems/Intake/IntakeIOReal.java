package frc.robot.subsystems.Intake;

import java.util.Map;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import prime.movers.LazyCANSparkMax;

public class IntakeIOReal implements IIntakeIO {

    private IIntakeIO intakeIO;
    private IntakeIOInputs intakeInputs = new IntakeIOInputs();
    private IntakeIOOutputs intakeOutputs = new IntakeIOOutputs();

    private DigitalInput m_topLimitSwitch;
    private DigitalInput m_bottomLimitSwitch;

    private LazyCANSparkMax m_rollers;
    private LazyCANSparkMax m_angleLeft;
    private LazyCANSparkMax m_angleRight;

    private PIDController m_anglePid;
    private Debouncer m_angleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public IntakeIOReal() {
        m_topLimitSwitch = new DigitalInput(IntakeSubsystem.VMap.TOP_LIMIT_SWITCH_CHANNEL);
        m_bottomLimitSwitch = new DigitalInput(IntakeSubsystem.VMap.BOTTOM_LIMIT_SWITCH_CHANNEL);

        m_rollers = new LazyCANSparkMax(IntakeSubsystem.VMap.ROLLER_CAN_ID, MotorType.kBrushless);
        m_rollers.restoreFactoryDefaults();
        m_rollers.setInverted(IntakeSubsystem.VMap.ROLLERS_INVERTED);
        m_rollers.setSmartCurrentLimit(40, 50);
        // m_rollers.setOpenLoopRampRate(0.250);

        m_angleLeft = new LazyCANSparkMax(IntakeSubsystem.VMap.NEO_LEFT_CAN_ID, MotorType.kBrushless);
        m_angleLeft.restoreFactoryDefaults();
        m_angleLeft.setInverted(IntakeSubsystem.VMap.NEO_LEFT_INVERTED);
        m_angleLeft.setSmartCurrentLimit(40, 60);

        m_angleRight = new LazyCANSparkMax(IntakeSubsystem.VMap.NEO_RIGHT_CAN_ID, MotorType.kBrushless);
        m_angleRight.restoreFactoryDefaults();
        m_angleRight.setInverted(IntakeSubsystem.VMap.NEO_RIGHT_INVERTED);
        m_angleRight.setSmartCurrentLimit(40, 60);

        intakeOutputs.m_angleStartPoint = intakeInputs.m_angleRightPosition;
        SmartDashboard.putNumber("Intake/AngleStartPoint", intakeOutputs.m_angleStartPoint);

        m_anglePid = IntakeSubsystem.VMap.INTAKE_ANGLE_PID.createPIDController(0.02);
        m_anglePid.setSetpoint(intakeOutputs.m_angleStartPoint);
        intakeOutputs.m_angleToggledIn = true;

    }

    @Override
    public IntakeIOInputs getInputs() {
       var inputs = new IntakeIOInputs();

       inputs.m_angleRightPosition = m_angleRight.getEncoder().getPosition();
       inputs.m_angleLeftPosition = m_angleLeft.getEncoder().getPosition();

       inputs.m_angleLeftState = m_angleLeft.get();
       inputs.m_angleRightState = m_angleRight.get();
       inputs.m_rollersState = m_rollers.get();

       inputs.m_topLimitSwitchState = m_topLimitSwitch.get();
       inputs.m_bottomLimitSwitchState = m_bottomLimitSwitch.get();

       inputs.m_anglePidOutput = m_anglePid.calculate(inputs.m_angleRightPosition, intakeOutputs.m_angleToggledIn ? intakeOutputs.m_angleStartPoint : (intakeOutputs.m_angleStartPoint - IntakeSubsystem.VMap.POSITION_DELTA));

       return inputs;
    }

    @Override
    public void setOutputs(IntakeIOOutputs outputs) {
        var inputs = getInputs();
    }

    @Override
    public void StopMotors() {
        m_angleLeft.stopMotor();
        m_angleRight.stopMotor();
    }

    @Override
    public void StopRollers() {
        m_rollers.stopMotor();
    }

    @Override
    public void runIntakeRollers(double speed) {
        m_rollers.set(speed);
    }

    @Override
    public void setAngleMotorSpeed(double speed) {
        m_angleLeft.set(-speed);
        m_angleRight.set(speed);
    }
    
}
