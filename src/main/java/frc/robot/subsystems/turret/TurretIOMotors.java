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
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static frc.robot.Constants.*;

public class TurretIOMotors implements TurretIO {

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

    private final TalonFX upperFlywheelMotor;
    private final MotionMagicVelocityVoltage upperFlywheelControl;

    private double targetFlywheelVelocity = 0;
    private double targetUpperFlywheelVelocity = 0;

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
    
    // Solver Buffer: Allow solver to see past 180 (for overshoot recovery)
    private static final double SOLVER_RANGE_BUFFER_DEG = 20.0; 

    // Status signals for low-latency reading
    private final StatusSignal<Angle> pos19Signal;
    private final StatusSignal<Angle> pos21Signal;
    private final StatusSignal<Angle> turretMotorPositionSignal;
    private final StatusSignal<Double> turretMotorPIDReferenceSignal;

    private final StatusSignal<AngularVelocity> hoodMotorVelocitySignal;
    private final StatusSignal<Voltage> hoodMotorVoltageSignal;
    private final StatusSignal<Current> hoodMotorCurrentSignal;
    private final StatusSignal<Angle> hoodEncoderPositionSignal;

    private final StatusSignal<AngularVelocity> flywheelMotorVelocitySignal;
    private final StatusSignal<Voltage> flywheelMotorVoltageSignal;
    private final StatusSignal<Current> flywheelMotorCurrentSignal;

    private final StatusSignal<AngularVelocity> upperFlywheelMotorVelocitySignal;
    private final StatusSignal<Voltage> upperFlywheelMotorVoltageSignal;
    private final StatusSignal<Current> upperFlywheelMotorCurrentSignal;
   

    public TurretIOMotors() {
        turretMotor = new TalonFX(CAN_ID_TURRET_MOTOR, frc.robot.RobotContainer.kCanivore);
        turretControl = new MotionMagicVoltage(0);

        hoodMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_HOOD_MOTOR, frc.robot.RobotContainer.kCanivore);
        hoodControl = new MotionMagicVoltage(0);

        flywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_MOTOR_FLYWHEEL, frc.robot.RobotContainer.kCanivore);
        flywheelControl = new MotionMagicVelocityVoltage(0);

        upperFlywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_UPPER_FLYWHEEL_MOTOR, frc.robot.RobotContainer.kCanivore);
        upperFlywheelControl = new MotionMagicVelocityVoltage(0);

        TalonFXConfiguration turretConfig = new TalonFXConfiguration();

        turretConfig.Slot0.kP = 300.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 1.0;

        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0; // turret rotations/sec
        turretConfig.MotionMagic.MotionMagicAcceleration = 3; // turret rotations/sec^2
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
        config19.MagnetSensor.MagnetOffset = -0.016602; // Set 19T Zero Offset
        enc19.getConfigurator().apply(config19);

        CANcoderConfiguration config21 = new CANcoderConfiguration();
        config21.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config21.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config21.MagnetSensor.MagnetOffset = 0.094238 ; // Set 21T Zero Offset
        enc21.getConfigurator().apply(config21);

        pos19Signal = enc19.getAbsolutePosition();
        pos21Signal = enc21.getAbsolutePosition();
        turretMotorPositionSignal = turretMotor.getPosition();
        turretMotorPIDReferenceSignal = turretMotor.getClosedLoopReference();

        BaseStatusSignal.refreshAll(pos19Signal, pos21Signal, turretMotorPositionSignal);

        double initialDegrees = calculateAbsolutePositionCRT();
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
        TalonFXConfiguration upperFlywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = 0.3;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;
        flywheelConfig.Slot0.kV = 0.12;
        flywheelConfig.Slot0.kA = 0.004;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelControl.Acceleration = 180.0;


        // Motor Revs per 1 Hood rev = ???
        // flywheelConfig.ClosedLoopGeneral.ContinuousWrap = false;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotorVelocitySignal = flywheelMotor.getVelocity();
        flywheelMotorVoltageSignal = flywheelMotor.getMotorVoltage();
        flywheelMotorCurrentSignal = flywheelMotor.getStatorCurrent();

        upperFlywheelMotor.getConfigurator().apply(upperFlywheelConfig);
        upperFlywheelMotorVelocitySignal = upperFlywheelMotor.getVelocity();
        upperFlywheelMotorVoltageSignal = flywheelMotor.getMotorVoltage();
        upperFlywheelMotorCurrentSignal = flywheelMotor.getStatorCurrent();
        

        flywheelConfig.Slot0.kP = 0.3;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;
        flywheelConfig.Slot0.kV = 0.0;
        flywheelConfig.Slot0.kA = 0.004;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelControl.Acceleration = 180.0;          
    
    }


    /**
     * Seeds the internal Falcon encoder using the 19T and 21T encoders.
     */
    public void seedTurretPosition() {
        double absoluteTurretDegrees = calculateAbsolutePositionCRT();

        // Convert degrees to rotations (0.5 = 180 deg)
        double turretRotations = absoluteTurretDegrees / 360.0;

        // Seed the Falcon 
        turretMotor.setPosition(turretRotations, 0.050);
        
        System.out.println("Turret Seeded at: " + absoluteTurretDegrees + " degrees");
    }



    /**
     * Solves for Absolute Position using CRT with the new variable names.
     */
    private double calculateAbsolutePositionCRT() {
        // Refresh signals to get latest data
        pos19Signal.refresh();
        pos21Signal.refresh();

        double pos1Rot = pos19Signal.getValueAsDouble(); 
        double pos2Rot = pos21Signal.getValueAsDouble();

        // Convert to 0-360 range for math
        double deg1 = pos1Rot * 360.0;
        double deg2 = pos2Rot * 360.0;

        double bestMatchAngle = 0.0;
        double minError = Double.MAX_VALUE;

        // Search range (-6 to +6 covers +/- 180 deg with margin)
        for (int k = -6; k <= 6; k++) {
            // Hypothesis: Turret Angle = (Enc1_Angle + k*360) / Ratio19
            double candidateTurretDeg = (deg1 + (k * 360.0)) / RATIO_19;

            // Check physical possibility (Soft Limit + Buffer)
            if (candidateTurretDeg < -(180.0 + SOLVER_RANGE_BUFFER_DEG) || 
                candidateTurretDeg > (180.0 + SOLVER_RANGE_BUFFER_DEG)) {
                continue;
            }

            // Validation: What should enc21 read?
            double totalEnc2Deg = candidateTurretDeg * RATIO_21;
            double expectedEnc2Deg = normalizeDegrees(totalEnc2Deg);

            // Compare Expected Enc2 vs Actual Enc2
            double errorDegrees = Math.abs(getShortestDistance(expectedEnc2Deg, deg2));

            if (errorDegrees < minError) {
                minError = errorDegrees;
                bestMatchAngle = candidateTurretDeg;
            }
        }

        // Convert Threshold (Rotations) to Degrees for check
        // 0.05 rot * 360 = 18 degrees
        double thresholdDegrees = MATCH_THRESHOLD * 360.0;

        if (minError > thresholdDegrees) {
            System.err.println("CRITICAL TURRET ERROR: CRT Solver mismatch. Error: " + minError);
            return 0.0; 
        }

        return bestMatchAngle;
    }

    private double normalizeDegrees(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    private double getShortestDistance(double a, double b) {
        double d = Math.abs(a - b) % 360.0; 
        return d > 180 ? 360 - d : d;
    }
    
    public void setTurretAngle(double degrees) {
        double clamped = Math.max(-180, Math.min(180, degrees));
        double targetRotations = clamped / 360.0;
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
        targetFlywheelVelocity = velocity;
        flywheelMotor.setControl(flywheelControl.withVelocity(velocity));
        upperFlywheelMotor.setControl(upperFlywheelControl.withVelocity(velocity));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            pos19Signal,
            pos21Signal,
            turretMotorPositionSignal,
            turretMotorPIDReferenceSignal,
            hoodMotorCurrentSignal,
            hoodMotorVelocitySignal,
            hoodMotorVoltageSignal,
            hoodEncoderPositionSignal,
            flywheelMotorVelocitySignal,
            upperFlywheelMotorVelocitySignal,
            upperFlywheelMotorVoltageSignal,
            upperFlywheelMotorCurrentSignal
        );

        double actualAngle = calculateAbsolutePositionCRT();
        inputs.turretAngle = Rotation2d.fromDegrees(actualAngle);
        inputs.turretPIDTargetAngle = turretAngleTarget;
        inputs.turretPIDActualAngle = actualAngle;
        inputs.turretMotorPIDTarget = turretMotorPIDReferenceSignal.getValueAsDouble();
        inputs.turretMotorRotations = turretMotorPositionSignal.getValueAsDouble();

        inputs.hoodTargetElevationPercent = hoodElevationTarget;
        inputs.hoodMotorVelocity = hoodMotorVelocitySignal.getValueAsDouble();
        inputs.hoodMotorVoltage = hoodMotorVoltageSignal.getValueAsDouble();
        inputs.hoodMotorCurrent = hoodMotorCurrentSignal.getValueAsDouble();
        inputs.hoodPIDActualAngle = hoodEncoderPositionSignal.getValueAsDouble();
        inputs.hoodPIDTargetAngle = hoodAngleTarget;

        inputs.enc19t = pos19Signal.getValueAsDouble(); 
        inputs.enc21t = pos21Signal.getValueAsDouble();

        

        inputs.flywheelMotorVelocity = flywheelMotorVelocitySignal.getValueAsDouble();
        inputs.flywheelMotorVoltage = flywheelMotorVoltageSignal.getValueAsDouble();
        inputs.flywheelMotorCurrent = flywheelMotorCurrentSignal.getValueAsDouble();
        inputs.flywheelPIDTargetVelocity = targetFlywheelVelocity;

        inputs.upperFlywheelMotorVelocity = upperFlywheelMotorVelocitySignal.getValueAsDouble();
        inputs.upperFlywheelMotorVoltage = upperFlywheelMotorVoltageSignal.getValueAsDouble();
        inputs.upperFlywheelMotorCurrent = upperFlywheelMotorCurrentSignal.getValueAsDouble();
        inputs.upperFlywheelPIDTargetVelocity = targetUpperFlywheelVelocity;        
    }

    public boolean areFlywheelsAtVelocity() {
        final double flywheelV = flywheelMotorVelocitySignal.getValueAsDouble();
        final double flywheelT = targetFlywheelVelocity;

        final double upperFlywheelV = upperFlywheelMotorVelocitySignal.getValueAsDouble();
        final double upperFlywheelT = targetFlywheelVelocity;

        return (targetFlywheelVelocity > 0 &&
            (Math.abs(flywheelV - flywheelT) < (0.05 * targetFlywheelVelocity)) && ((targetUpperFlywheelVelocity > 0) 
            && (Math.abs(upperFlywheelV - upperFlywheelT) < (0.05 * targetFlywheelVelocity))));
    }
}
