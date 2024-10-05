package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.PwmLEDs;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.BlinkPattern;
import prime.control.LEDs.Patterns.ChasePattern;
import prime.control.LEDs.Patterns.SolidPattern;

public class ShooterIOReal implements IShooterIO{
    private PwmLEDs m_leds;
    private TalonFX m_talonFX;
    private VictorSPX m_victorSPX;
    private DoubleSolenoid m_elevationSolenoid;
    private DigitalInput m_noteDetector;

    public ShooterIOReal(PwmLEDs leds) {
        m_leds = leds;

        m_talonFX = new TalonFX(ShooterSubsystem.VMap.TALONFX_CAN_ID);
        m_talonFX.getConfigurator().apply(new TalonFXConfiguration());
        m_talonFX.setInverted(true);
        m_talonFX.setNeutralMode(NeutralModeValue.Brake);

        m_victorSPX = new VictorSPX(ShooterSubsystem.VMap.VICTORSPX_CAN_ID);
        m_victorSPX.configFactoryDefault();
        m_victorSPX.setNeutralMode(NeutralMode.Brake);

        m_elevationSolenoid =
          new DoubleSolenoid(
            30,
            PneumaticsModuleType.REVPH,
            ShooterSubsystem.VMap.ELEVATION_SOLENOID_FORWARD_CHANNEL,
            ShooterSubsystem.VMap.ELEVATION_SOLENOID_REVERSE_CHANNEL
          );

        m_noteDetector = new DigitalInput(ShooterSubsystem.VMap.NOTE_DETECTOR_DIO_CHANNEL);
    }

    @Override
    public ShooterIOInputs getInputs() {
        var inputs = new ShooterIOInputs();

        inputs.m_noteDetectorState = m_noteDetector.get();

        inputs.m_talonFXVelocity = m_talonFX.getVelocity().getValueAsDouble();
        inputs.m_talonFXState = m_talonFX.get();

        inputs.m_victorSPXOutputPercent = m_victorSPX.getMotorOutputPercent();
        
        inputs.m_elevationSolenoidState = m_elevationSolenoid.get();

        return inputs;
    }

    @Override
    public void setOutputs(ShooterIOOutputs outputs) {
        var inputs = new ShooterIOInputs();

        if (outputs.m_stopTalonFX) {
            m_talonFX.stopMotor();
            outputs.m_stopTalonFX = false;
        } else {
            m_talonFX.set(outputs.m_talonFXSpeed);
        }

        if (outputs.m_stopVictorSPX) {
            m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);
            outputs.m_stopVictorSPX = false;
        } else {
            m_victorSPX.set(VictorSPXControlMode.PercentOutput, outputs.m_victorSPXSpeed);
        }

        //#region LED Logic
        if ((outputs.m_stopTalonFX && outputs.m_stopVictorSPX) || inputs.m_elevationSolenoidState == Value.kReverse) {
            m_leds.restorePersistentStripPattern();
        }

        if (inputs.m_elevationSolenoidState == Value.kForward) {
            m_leds.setStripTemporaryPattern(new SolidPattern(Color.WHITE));
        }

        if (outputs.m_playNoteDetectedLedPattern) {
            m_leds.setStripTemporaryPattern(new BlinkPattern(prime.control.LEDs.Color.ORANGE, 0.2));
        }

        if (outputs.m_startShootingNoteLedPattern) {
            m_leds.setStripTemporaryPattern(new ChasePattern(Color.GREEN, 0.25, !inputs.m_noteDetectorState));
        }
        //#endregion

         m_elevationSolenoid.set(outputs.m_elevationSolenoidValue);

    }

}