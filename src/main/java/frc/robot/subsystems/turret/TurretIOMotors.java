package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


import static frc.robot.Constants.*;

public class TurretIOMotors implements TurretIO {

    private double turretAngleTarget;
    private double hoodElevationTarget; //0 .. 100
    private double hoodAngleTarget; //0 .. 0.9

    private final CANcoder enc31; // The encoder on the 31T gear
    private final CANcoder enc37; // The encoder on the 37T gear
    private final CANcoder encHood;

    private final SparkMax turretMotor;
    private final SparkClosedLoopController turretPID;

    private final TalonFX hoodMotor;
    private final MotionMagicVoltage hoodControl;

    private final TalonFX flywheelMotor;
    private final MotionMagicVelocityVoltage flywheelControl;

    private final TalonFX upperFlywheelMotor;
    private final MotionMagicVelocityVoltage upperFlywheelControl;

    private double targetFlywheelVelocity = 0;
    private double targetUpperFlywheelVelocity = 0;

    // Gear Constants
    private static final double MAIN_TEETH = 125.0;
    private static final double TEETH_31 = 31.0;
    private static final double TEETH_37 = 37.0;

    // Gear Ratios (Encoder Rotations per Turret Rotation)
    private static final double RATIO_31 = MAIN_TEETH / TEETH_31; // ~7.258
    private static final double RATIO_37 = MAIN_TEETH / TEETH_37; // ~6.081

    // New Turret Motor Gear Ratio (Assuming previous 5:1 planetary and new 125/18 pinion) = 34.72222222222222222
    private static final double TURRET_GEAR_RATIO = 34.7222;

    // How close the two encoders must match to be considered valid (in rotations)
    // 0.05 rotations is 18 degrees of the encoder shaft, which is plenty of margin
    // for backlash
    private static final double MATCH_THRESHOLD = 0.05;
    
    // Solver Buffer: Allow solver to see past 180 (for overshoot recovery)
    private static final double SOLVER_RANGE_BUFFER_DEG = 20.0; 

    // Status signals for low-latency reading
    private final StatusSignal<Angle> pos31Signal;
    private final StatusSignal<Angle> pos37Signal;

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
        turretMotor = new SparkMax(CAN_ID_TURRET_MOTOR, MotorType.kBrushless);
        turretPID = turretMotor.getClosedLoopController();

        SparkMaxConfig turretConfig = new SparkMaxConfig();

        turretConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.1) // Note: Requires retuning for SparkMax
            .i(0.0)
            .d(0.0)
            .positionWrappingEnabled(false);

        // Configure mechanism conversions to rotate in Turret Rotations
        turretConfig.encoder
            .positionConversionFactor(1.0 / TURRET_GEAR_RATIO)
            .velocityConversionFactor((1.0 / TURRET_GEAR_RATIO) / 60.0);

        // Set limits to +/- 190 degrees to allow full 360 coverage with overlap
        turretConfig.softLimit
            .forwardSoftLimit(0.53)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-0.53)
            .reverseSoftLimitEnabled(true);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_HOOD_MOTOR, frc.robot.RobotContainer.kCanivore);
        hoodControl = new MotionMagicVoltage(0);

        flywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_MOTOR_FLYWHEEL, frc.robot.RobotContainer.kCanivore);
        flywheelControl = new MotionMagicVelocityVoltage(0);

        upperFlywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_UPPER_FLYWHEEL_MOTOR, frc.robot.RobotContainer.kCanivore);
        upperFlywheelControl = new MotionMagicVelocityVoltage(0);

        enc31 = new CANcoder(CAN_ID_TURRET_ENCODER_31T, frc.robot.RobotContainer.kCanivore);
        enc37 = new CANcoder(CAN_ID_TURRET_ENCODER_37T, frc.robot.RobotContainer.kCanivore);

        CANcoderConfiguration config31 = new CANcoderConfiguration();
        config31.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config31.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config31.MagnetSensor.MagnetOffset = -0.016602; // Retained 19T Zero Offset for 31T position
        enc31.getConfigurator().apply(config31);

        CANcoderConfiguration config37 = new CANcoderConfiguration();
        config37.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config37.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config37.MagnetSensor.MagnetOffset = 0.094238 ; // Retained 21T Zero Offset for 37T position
        enc37.getConfigurator().apply(config37);

        pos31Signal = enc31.getAbsolutePosition();
        pos37Signal = enc37.getAbsolutePosition();

        BaseStatusSignal.refreshAll(pos31Signal, pos37Signal);

        double initialDegrees = calculateAbsolutePositionCRT();
        if (initialDegrees != -1.0) {
            // Force the seed to be within -180 to 180
            if (initialDegrees > 180)
                initialDegrees -= 360;
            turretMotor.getEncoder().setPosition(initialDegrees / 360.0);
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

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotorVelocitySignal = flywheelMotor.getVelocity();
        flywheelMotorVoltageSignal = flywheelMotor.getMotorVoltage();
        flywheelMotorCurrentSignal = flywheelMotor.getStatorCurrent();

        upperFlywheelMotor.getConfigurator().apply(upperFlywheelConfig);
        upperFlywheelMotorVelocitySignal = upperFlywheelMotor.getVelocity();
        upperFlywheelMotorVoltageSignal = upperFlywheelMotor.getMotorVoltage();
        upperFlywheelMotorCurrentSignal = upperFlywheelMotor.getStatorCurrent();
        
        upperFlywheelConfig.Slot0.kP = 0.3;
        upperFlywheelConfig.Slot0.kI = 0.0;
        upperFlywheelConfig.Slot0.kD = 0.0;
        upperFlywheelConfig.Slot0.kV = 0.0;
        upperFlywheelConfig.Slot0.kA = 0.004;

        upperFlywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        upperFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        upperFlywheelControl.Acceleration = 180.0;          
    }

    /**
     * Seeds the internal NEO encoder using the 31T and 37T encoders.
     */
    public void seedTurretPosition() {
        double absoluteTurretDegrees = calculateAbsolutePositionCRT();

        // Convert degrees to rotations (0.5 = 180 deg)
        double turretRotations = absoluteTurretDegrees / 360.0;

        // Seed the SparkMax 
        turretMotor.getEncoder().setPosition(turretRotations);
        
        System.out.println("Turret Seeded at: " + absoluteTurretDegrees + " degrees");
    }

    /**
     * Solves for Absolute Position using CRT with the new variable names.
     */
    private double calculateAbsolutePositionCRT() {
        // Refresh signals to get latest data
        pos31Signal.refresh();
        pos37Signal.refresh();

        double pos1Rot = pos31Signal.getValueAsDouble(); 
        double pos2Rot = pos37Signal.getValueAsDouble();

        // Convert to 0-360 range for math
        double deg1 = pos1Rot * 360.0;
        double deg2 = pos2Rot * 360.0;

        double bestMatchAngle = 0.0;
        double minError = Double.MAX_VALUE;

        // Search range (-15 to +15 covers +/- 180 deg with margin for new ratios)
        for (int k = -15; k <= 15; k++) {
            // Hypothesis: Turret Angle = (Enc1_Angle + k*360) / Ratio31
            double candidateTurretDeg = (deg1 + (k * 360.0)) / RATIO_31;

            // Check physical possibility (Soft Limit + Buffer)
            if (candidateTurretDeg < -(180.0 + SOLVER_RANGE_BUFFER_DEG) || 
                candidateTurretDeg > (180.0 + SOLVER_RANGE_BUFFER_DEG)) {
                continue;
            }

            // Validation: What should enc37 read?
            double totalEnc2Deg = candidateTurretDeg * RATIO_37;
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
        turretAngleTarget = degrees;
        double clamped = Math.max(-180, Math.min(180, degrees));
        double targetRotations = clamped / 360.0;
        turretPID.setSetpoint(targetRotations, ControlType.kPosition);
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
            pos31Signal,
            pos37Signal,
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
        inputs.turretMotorPIDTarget = turretAngleTarget / 360.0; // in rotations
        inputs.turretMotorRotations = turretMotor.getEncoder().getPosition(); // Configured to return rotations

        inputs.hoodTargetElevationPercent = hoodElevationTarget;
        inputs.hoodMotorVelocity = hoodMotorVelocitySignal.getValueAsDouble();
        inputs.hoodMotorVoltage = hoodMotorVoltageSignal.getValueAsDouble();
        inputs.hoodMotorCurrent = hoodMotorCurrentSignal.getValueAsDouble();
        inputs.hoodPIDActualAngle = hoodEncoderPositionSignal.getValueAsDouble();
        inputs.hoodPIDTargetAngle = hoodAngleTarget;

        inputs.enc31t = pos31Signal.getValueAsDouble(); 
        inputs.enc37t = pos37Signal.getValueAsDouble();

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