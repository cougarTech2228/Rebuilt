package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.Constants.*;

public class TurretIOTalonFX implements TurretIO {

    private double turretAngleTarget;
    private double hoodElevationTarget; //0 .. 100
    private double hoodAngleTarget; //0 .. 0.9

    private final CANcoder enc19; // The encoder on the 19T gear
    private final CANcoder enc21; // The encoder on the 21T gear
    private final CANcoder encHood;

    private final TalonFX turretMotor;
    private final MotionMagicVoltage turretControl;
    private final TalonFX hoodMotor;
    private final MotionMagicVoltage hoodControl;
    private final TalonFX flywheelMotor;
    private final MotionMagicVelocityVoltage flywheelControl;

    // Gear Constants
    private static final double MAIN_TEETH = 200.0;
    private static final double TEETH_19 = 19.0;
    private static final double TEETH_21 = 21.0;

    // Gear Ratios (Encoder Rotations per Turret Rotation)
    private static final double RATIO_19 = MAIN_TEETH / TEETH_19; // ~10.526
    private static final double RATIO_21 = MAIN_TEETH / TEETH_21; // ~9.523

    // How close the two encoders must match to be considered valid (in rotations)
    // 0.05 rotations is 18 degrees of the encoder shaft, which is plenty of margin
    // for backlash
    private static final double MATCH_THRESHOLD = 0.05;

    // Status signals for low-latency reading
    private final StatusSignal<Angle> pos19Signal;
    private final StatusSignal<Angle> pos21Signal;
    private final StatusSignal<Angle> turretMotorPositionSignal;

    private final StatusSignal<AngularVelocity> hoodMotorVelocitySignal;
    private final StatusSignal<Voltage> hoodMotorVoltageSignal;
    private final StatusSignal<Current> hoodMotorCurrentSignal;
    private final StatusSignal<Angle> hoodEncoderPositionSignal;

    public TurretIOTalonFX() {
        turretMotor = new TalonFX(CAN_ID_TURRET_MOTOR, frc.robot.RobotContainer.kCanivore);
        turretControl = new MotionMagicVoltage(0);
        hoodMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_HOOD_MOTOR, frc.robot.RobotContainer.kCanivore);
        hoodControl = new MotionMagicVoltage(0);
        flywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_MOTOR_FLYWHEEL, frc.robot.RobotContainer.kCanivore);
        flywheelControl = new MotionMagicVelocityVoltage(0);

        TalonFXConfiguration turretConfig = new TalonFXConfiguration();

        turretConfig.Slot0.kP = 300.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 1.0;

        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0; // turret rotations/sec
        turretConfig.MotionMagic.MotionMagicAcceleration = 10.0; // turret rotations/sec^2
        turretConfig.MotionMagic.MotionMagicJerk = 0.0;

        // Motor Revs per 1 Turret Rev = 5 * (200/18) = 55.555...
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretConfig.Feedback.SensorToMechanismRatio = 55.55555555555556;
        turretConfig.ClosedLoopGeneral.ContinuousWrap = false;

        // Invert motor so positive voltage moves CCW (Standard Robot Frame)
        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Set limits to +/- 190 degrees to allow full 360 coverage with overlap
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.53;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.53;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        turretMotor.getConfigurator().apply(turretConfig);

        enc19 = new CANcoder(CAN_ID_TURRET_ENCODER_19T, frc.robot.RobotContainer.kCanivore);
        enc21 = new CANcoder(CAN_ID_TURRET_ENCODER_21T, frc.robot.RobotContainer.kCanivore);

        CANcoderConfiguration config19 = new CANcoderConfiguration();
        config19.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config19.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config19.MagnetSensor.MagnetOffset = -0.070068; // Set 19T Zero Offset
        enc19.getConfigurator().apply(config19);

        CANcoderConfiguration config21 = new CANcoderConfiguration();
        config21.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config21.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config21.MagnetSensor.MagnetOffset = -0.032715; // Set 21T Zero Offset
        enc21.getConfigurator().apply(config21);

        pos19Signal = enc19.getAbsolutePosition();
        pos21Signal = enc21.getAbsolutePosition();
        turretMotorPositionSignal = turretMotor.getPosition();

        BaseStatusSignal.refreshAll(pos19Signal, pos21Signal, turretMotorPositionSignal);

        double initialDegrees = getTurretPositionDegrees();
        if (initialDegrees != -1.0) {
            // Force the seed to be within -180 to 180
            if (initialDegrees > 180)
                initialDegrees -= 360;
            turretMotor.setPosition(initialDegrees / 360.0);
        }

        encHood = new CANcoder(CAN_ID_TURRET_HOOD_ENCODER, frc.robot.RobotContainer.kCanivore);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        hoodConfig.Slot0.kP = 75.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kV = 0.1;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0; // hood rotations/sec
        hoodConfig.MotionMagic.MotionMagicAcceleration = 500.0; // hood rotations/sec^2
        hoodConfig.MotionMagic.MotionMagicJerk = 0.0;

        // Motor Revs per 1 Hood rev = ???
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hoodConfig.Feedback.FeedbackRemoteSensorID = CAN_ID_TURRET_HOOD_ENCODER;
        hoodConfig.Feedback.RotorToSensorRatio = 50.0;
        hoodConfig.ClosedLoopGeneral.ContinuousWrap = false;
        // Set limits to safe values
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.9;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodMotor.getConfigurator().apply(hoodConfig);

        CANcoderConfiguration configHood = new CANcoderConfiguration();
        configHood.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        configHood.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        configHood.MagnetSensor.MagnetOffset = 0.082764;
        encHood.getConfigurator().apply(configHood);

        hoodMotorVelocitySignal = hoodMotor.getVelocity();
        hoodMotorVoltageSignal = hoodMotor.getMotorVoltage();
        hoodMotorCurrentSignal = hoodMotor.getStatorCurrent();
        hoodEncoderPositionSignal = encHood.getPosition();

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = 0.3;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;
        flywheelConfig.Slot0.kV = 0.12;
        flywheelConfig.Slot0.kA = 0.004;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelControl.Acceleration = 120.0;


        // Motor Revs per 1 Hood rev = ???
        // flywheelConfig.ClosedLoopGeneral.ContinuousWrap = false;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
    }

    public double getTurretPositionDegrees() {
        // Get normalized positions (0.0 to 1.0)
        double p19 = normalize(pos19Signal.getValue().in(Rotations));
        double p21 = normalize(pos21Signal.getValue().in(Rotations));

        // Search wider range (-1 to +12) to handle wrap-around correctly
        int minTurns = -1;
        int maxTurns = (int) Math.ceil(RATIO_19) + 1;

        double bestError = 1.0;
        double bestPosition = -1.0;

        for (int k = minTurns; k <= maxTurns; k++) {
            double candidateTurretRotations = (k + p19) / RATIO_19;
            double expectedP21Total = candidateTurretRotations * RATIO_21;
            double expectedP21 = normalize(expectedP21Total);

            double error = getShortestDistance(expectedP21, p21);

            // Find Minimum Error
            if (error < bestError) {
                bestError = error;
                bestPosition = candidateTurretRotations;
            }
        }

        if (bestError < MATCH_THRESHOLD) {
            double degrees = (bestPosition * 360.0) % 360.0;
            if (degrees < 0)
                degrees += 360.0;

            // Invert the result so that CCW is Positive (Standard Robot Frame)
            return (360.0 - degrees) % 360.0;
        }

        System.err.println("Turret Sync Failed. Best Error: " + bestError);
        return -1.0;
    }

    private double normalize(double input) {
        double value = input % 1.0;
        if (value < 0)
            value += 1.0;
        return value;
    }

    private double getShortestDistance(double a, double b) {
        double diff = Math.abs(a - b);
        if (diff > 0.5)
            return 1.0 - diff;
        return diff;
    }

    @Override
    public void setTurretAngle(double turretAngle) {
        turretAngleTarget = turretAngle;

        // 1. Wrap the input to -180 to 180
        double wrappedDegrees = turretAngleTarget % 360;
        if (wrappedDegrees > 180) wrappedDegrees -= 360;
        if (wrappedDegrees < -180) wrappedDegrees += 360;

        // 2. Convert to rotations (-0.5 to 0.5)
        double targetRotations = wrappedDegrees / 360.0;
        
        // 3. Command the motor
        // Since wrap is off, if the motor is at 0.45 and target is -0.45, 
        // it will rotate the long way back through 0 to avoid the wire limit.
        turretMotor.setControl(turretControl.withPosition(targetRotations));
    }

    @Override
    //0-100 from minimum elevation to maximum elevation
    public void setHoodAngle(double hoodElevation) {
        hoodElevationTarget = hoodElevation; // Elevation 0 .. 100
        hoodAngleTarget = hoodElevation / 100 * 0.9; // Angle 0 .. 0.9

        hoodMotor.setControl(hoodControl.withPosition(hoodAngleTarget));
    }

    @Override
    public void setFlywheelVelocity(double velocity) {
        flywheelMotor.setControl(flywheelControl.withVelocity(velocity));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            pos19Signal,
            pos21Signal,
            turretMotorPositionSignal,
            hoodMotorCurrentSignal,
            hoodMotorVelocitySignal,
            hoodMotorVoltageSignal,
            hoodEncoderPositionSignal
        );

        double actualAngle = getTurretPositionDegrees();
        inputs.turretAngle = Rotation2d.fromDegrees(actualAngle);
        inputs.turretPIDTargetAngle = turretAngleTarget;
        inputs.turretPIDActualAngle = actualAngle;

        inputs.hoodTargetElevationPercent = hoodElevationTarget;
        inputs.hoodMotorVelocity = hoodMotorVelocitySignal.getValueAsDouble();
        inputs.hoodMotorVoltage = hoodMotorVoltageSignal.getValueAsDouble();
        inputs.hoodMotorCurrent = hoodMotorCurrentSignal.getValueAsDouble();
        inputs.hoodPIDActualAngle = hoodEncoderPositionSignal.getValueAsDouble();
        inputs.hoodPIDTargetAngle = hoodAngleTarget;
    }
}
