package frc.robot.subsystems.Intake;

import java.util.Map;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import prime.movers.LazyCANSparkMax;

public class IntakeIOReal implements IIntakeIO {

    private LazyCANSparkMax m_rollers;
    private LazyCANSparkMax m_angleLeft;
    private LazyCANSparkMax m_angleRight;

    private PIDController m_anglePid;
    private Debouncer m_angleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public IntakeIOReal() {
        m_topLimitSwitch = new DigitalInput(IntakeSubsystem.Map.TOP_LIMIT_SWITCH_CHANNEL);
    m_bottomLimitSwitch = new DigitalInput(IntakeSubsystem.Map.BOTTOM_LIMIT_SWITCH_CHANNEL);

    m_rollers = new LazyCANSparkMax(IntakeSubsystem.Map.ROLLER_CAN_ID, MotorType.kBrushless);
    m_rollers.restoreFactoryDefaults();
    m_rollers.setInverted(IntakeSubsystem.Map.ROLLERS_INVERTED);
    m_rollers.setSmartCurrentLimit(40, 50);
    // m_rollers.setOpenLoopRampRate(0.250);

    m_angleLeft = new LazyCANSparkMax(IntakeSubsystem.Map.NEO_LEFT_CAN_ID, MotorType.kBrushless);
    m_angleLeft.restoreFactoryDefaults();
    m_angleLeft.setInverted(IntakeSubsystem.Map.NEO_LEFT_INVERTED);
    m_angleLeft.setSmartCurrentLimit(40, 60);

    m_angleRight = new LazyCANSparkMax(IntakeSubsystem.Map.NEO_RIGHT_CAN_ID, MotorType.kBrushless);
    m_angleRight.restoreFactoryDefaults();
    m_angleRight.setInverted(IntakeSubsystem.Map.NEO_RIGHT_INVERTED);
    m_angleRight.setSmartCurrentLimit(40, 60);

    m_angleStartPoint = getPositionRight();
    SmartDashboard.putNumber("Intake/AngleStartPoint", m_angleStartPoint);

    m_anglePid = IntakeSubsystem.Map.INTAKE_ANGLE_PID.createPIDController(0.02);
    m_anglePid.setSetpoint(m_angleStartPoint);
    m_angleToggledIn = true;

    }

    @Override
    public IntakeIOInputs getInputs() {
       var inputs = new IntakeIOInputs();

       inputs.m_angleRightPosition = m_angleRight.getEncoder().getPosition();
       inputs.m_angleLeftPosition = m_angleLeft.getEncoder().getPosition();

       return inputs;
    }

    @Override
    public void setOutputs(IntakeIOOutputs outputs) {
        var inputs = getInputs();


        // Controls the speed of the motors in the Intake

        if (outputs.m_stopAngleLeft) {
            m_angleLeft.stopMotor();
        } else {
            m_angleLeft.set(outputs.m_angleLeftSpeed);
        }

        if (outputs.m_stopAngleRight) {
            m_angleRight.stopMotor();
        } else {
            m_angleRight.set(outputs.m_angleRightSpeed);
        }

        if (outputs.m_stopRollers) {
            m_rollers.stopMotor();
        } else {
            m_rollers.set(outputs.m_rollersSpeed);
        }
    }
    
}
