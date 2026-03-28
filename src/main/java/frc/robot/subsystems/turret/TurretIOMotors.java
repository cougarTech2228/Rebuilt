package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static frc.robot.Constants.*;

public class TurretIOMotors implements TurretIO {

    private double turretPIDTarget;
    private double turretRealTarget;
    private double hoodElevationTarget; //0 .. 100

    private final CANcoder enc31; // The encoder on the 31T gear
    private final CANcoder enc37; // The encoder on the 37T gear
    private final CANcoder encHood;

    private final TalonFXS turretMotor;
    private final MotionMagicVoltage turretControl;

    private final SparkMax hoodMotor;
    private final SparkClosedLoopController hoodPID;

    private final TalonFX flywheelMotor;
    private final MotionMagicVelocityVoltage flywheelControl;

    private final TalonFX upperFlywheelMotor;
    private final MotionMagicVelocityVoltage upperFlywheelControl;

    private double targetFlywheelVelocity = 0;
    private double targetUpperFlywheelVelocity = 0;

    // Gear Constants
    private static final double MAIN_TEETH_ENCODER = 160.0; 
    private static final double MAIN_TEETH_BELT = 125.0; 
    private static final double TEETH_31 = 31.0;
    private static final double TEETH_37 = 37.0;

    // Gear Ratios (Encoder Rotations per Turret Rotation)
    private static final double RATIO_31 = MAIN_TEETH_ENCODER / TEETH_31; // Exactly 5.16129
    private static final double RATIO_37 = MAIN_TEETH_ENCODER / TEETH_37; // ~4.32432

    // Turret Motor Gear Ratio (10:1 planetary and 125/18 pinion) -> ~69.444
    private static final double TURRET_MOTOR_GEARBOX_RATIO = 10.0;
    private static final double TURRET_GEAR_RATIO = (MAIN_TEETH_BELT / 18.0) * TURRET_MOTOR_GEARBOX_RATIO;

    private static final double TURRET_MOTOR_REVS_PER_DEGREE = (TURRET_GEAR_RATIO / 360);

    // How close the two encoders must match to be considered valid (in rotations)
    // Increased to 0.10 (36 degrees) to absorb physical belt/gear backlash of ~19 degrees
    private static final double MATCH_THRESHOLD = 0.10;

    // degrees of error allowed for turret PID aiming
    private static final double ALLOWED_TURRET_ERROR_DEGREES = 5.0;
    private static final double ALLOWED_TURRET_ERROR_ROTATIONS = (ALLOWED_TURRET_ERROR_DEGREES * TURRET_MOTOR_REVS_PER_DEGREE);

    // Status signals for low-latency reading
    private final StatusSignal<Angle> pos31Signal;
    private final StatusSignal<Angle> pos37Signal;

    private final StatusSignal<Current> turretMotorCurrentSignal;
    private final StatusSignal<Angle> turretMotorPositionSignal;

    // private final StatusSignal<Current> hoodMotorCurrentSignal;
    private final StatusSignal<Angle> hoodEncoderPositionSignal;
    // private final StatusSignal<Angle> hoodMotorPositionSignal;

    private final StatusSignal<AngularVelocity> flywheelMotorVelocitySignal;
    private final StatusSignal<Voltage> flywheelMotorVoltageSignal;
    private final StatusSignal<Current> flywheelMotorCurrentSignal;

    private final StatusSignal<AngularVelocity> upperFlywheelMotorVelocitySignal;
    private final StatusSignal<Voltage> upperFlywheelMotorVoltageSignal;
    private final StatusSignal<Current> upperFlywheelMotorCurrentSignal;
   
    private final BaseStatusSignal[] allSignals;

    public TurretIOMotors() {
        turretMotor = new TalonFXS(CAN_ID_TURRET_MOTOR, RobotContainer.kRio);
        turretControl = new MotionMagicVoltage(0);

        TalonFXSConfiguration turretConfig = new TalonFXSConfiguration();
        turretConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        turretConfig.Slot0.kP = 2;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;
        turretConfig.Slot0.kV = 0.0005;

        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 1000; // rotations/sec
        turretConfig.MotionMagic.MotionMagicAcceleration = 100 * 4;  // rotations/sec/sec
        turretConfig.MotionMagic.MotionMagicJerk = 0.0;

        turretConfig.ClosedLoopGeneral.ContinuousWrap = false;

        // Set limits to safe values
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (float)((TurretConstants.TURRET_MAX_ROTATION / 360.0) * TURRET_GEAR_RATIO);
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (float)((TurretConstants.TURRET_MIN_ROTATION / 360.0) * TURRET_GEAR_RATIO);
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        turretMotor.getConfigurator().apply(turretConfig);

        turretMotorCurrentSignal = turretMotor.getTorqueCurrent();
        turretMotorPositionSignal = turretMotor.getPosition();

        hoodMotor = new SparkMax(Constants.CAN_ID_TURRET_HOOD_MOTOR, MotorType.kBrushless);
        hoodPID = hoodMotor.getClosedLoopController();

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.5)
            .i(0)
            .d(0)
            .feedForward.kV(0.000);
        hoodConfig.closedLoop.maxMotion.maxAcceleration(3000);
        hoodConfig.closedLoop.maxMotion.cruiseVelocity(2000);
        hoodConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        hoodConfig.closedLoop.maxMotion.allowedProfileError(1);
        hoodMotor.configure(hoodConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);


        flywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_MOTOR_FLYWHEEL, frc.robot.RobotContainer.kRio);
        flywheelControl = new MotionMagicVelocityVoltage(0);

        upperFlywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_UPPER_FLYWHEEL_MOTOR, frc.robot.RobotContainer.kRio);
        upperFlywheelControl = new MotionMagicVelocityVoltage(0);

        enc31 = new CANcoder(CAN_ID_TURRET_ENCODER_31T, frc.robot.RobotContainer.kRio);
        enc37 = new CANcoder(CAN_ID_TURRET_ENCODER_37T, frc.robot.RobotContainer.kRio);

        CANcoderConfiguration config31 = new CANcoderConfiguration();
        config31.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config31.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config31.MagnetSensor.MagnetOffset = TurretConstants.ENCODER_31T_OFFSET;
        enc31.getConfigurator().apply(config31);

        CANcoderConfiguration config37 = new CANcoderConfiguration();
        config37.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config37.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config37.MagnetSensor.MagnetOffset = TurretConstants.ENCODER_37T_OFFSET;
        enc37.getConfigurator().apply(config37);

        pos31Signal = enc31.getAbsolutePosition();
        pos37Signal = enc37.getAbsolutePosition();

        BaseStatusSignal.refreshAll(pos31Signal, pos37Signal);

        seedTurretPosition();

        encHood = new CANcoder(CAN_ID_TURRET_HOOD_ENCODER, frc.robot.RobotContainer.kRio);
        CANcoderConfiguration configHood = new CANcoderConfiguration();
        configHood.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        configHood.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        configHood.MagnetSensor.MagnetOffset = TurretConstants.ENCODER_HOOD_MAGNET_OFFSET;
        encHood.getConfigurator().apply(configHood);

        hoodEncoderPositionSignal = encHood.getPosition();

        seedHoodAngle();

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        TalonFXConfiguration upperFlywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = 0.1;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;
        flywheelConfig.Slot0.kV = 0.118;
        flywheelConfig.Slot0.kA = 0.0;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelControl.Acceleration = 200.0;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotorVelocitySignal = flywheelMotor.getVelocity();
        flywheelMotorVoltageSignal = flywheelMotor.getMotorVoltage();
        flywheelMotorCurrentSignal = flywheelMotor.getStatorCurrent();

        upperFlywheelConfig.Slot0.kP = 0.3;
        upperFlywheelConfig.Slot0.kI = 0.0;
        upperFlywheelConfig.Slot0.kD = 0.0;
        upperFlywheelConfig.Slot0.kV = 0.1;
        upperFlywheelConfig.Slot0.kA = 0.0;

        upperFlywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        upperFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        upperFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        upperFlywheelControl.Acceleration = 200.0;          
        
        upperFlywheelMotor.getConfigurator().apply(upperFlywheelConfig);
        upperFlywheelMotorVelocitySignal = upperFlywheelMotor.getVelocity();
        upperFlywheelMotorVoltageSignal = upperFlywheelMotor.getMotorVoltage();
        upperFlywheelMotorCurrentSignal = upperFlywheelMotor.getStatorCurrent();

        allSignals = new BaseStatusSignal[] {
            pos31Signal,
            pos37Signal,
            turretMotorCurrentSignal,
            turretMotorPositionSignal,
            hoodEncoderPositionSignal,
            flywheelMotorVelocitySignal,
            upperFlywheelMotorVelocitySignal,
            upperFlywheelMotorVoltageSignal,
            upperFlywheelMotorCurrentSignal
        };
    }

    /**
     * Seeds the internal NEO encoder using the 31T and 37T encoders.
     */
    public void seedTurretPosition() {
        double absoluteTurretDegrees = calculateAbsolutePositionCRT();

        // Convert degrees to mechanism rotations (0.5 = 180 deg)
        double turretRotations = absoluteTurretDegrees / 360.0;

        // Seed the turret in Motor Rotations
        double motorRotations = turretRotations * TURRET_GEAR_RATIO;
        turretMotor.setPosition(motorRotations);
        
        System.out.println("Turret Seeded at: " + absoluteTurretDegrees + " degrees == " + motorRotations);
    }

    private void seedHoodAngle() {
        double angle = hoodEncoderPositionSignal.waitForUpdate(0.5).getValueAsDouble();
        double val = angle * TurretConstants.HOOD_GEAR_RATIO;
        hoodMotor.getEncoder().setPosition(val);
    }

    /**
     * Solves for Absolute Position using CRT.
     */
    private double calculateAbsolutePositionCRT() {
        // Refresh signals to get latest data
        pos31Signal.refresh();
        pos37Signal.refresh();

        // Normalize to 0..360 natively right away to prevent ANY negative modulo weirdness
        double deg1 = normalizeDegrees(pos31Signal.getValueAsDouble() * 360.0);
        double deg2 = normalizeDegrees(pos37Signal.getValueAsDouble() * 360.0);

        double bestMatchAngle = 0.0;
        double minError = Double.MAX_VALUE;

        // Try each plausible number of full enc31 wraps. Range -10 to 10 covers massive travel margin.
        for (int k = -10; k <= 10; k++) {
            // Unwrap enc31 by k full rotations, then back-calculate the implied turret angle.
            double candidateTurretDeg = (deg1 + (k * 360.0)) / RATIO_31;

            if (candidateTurretDeg < TurretConstants.TURRET_MIN_ROTATION - 40.0 ||
                candidateTurretDeg > TurretConstants.TURRET_MAX_ROTATION + 40.0) {
                continue;
            }

            // If the turret is really at candidateTurretDeg, predict what enc37 should read.
            double expectedEnc2Deg = normalizeDegrees(candidateTurretDeg * RATIO_37);

            // Score this candidate by how closely enc37's actual reading matches the prediction.
            double errorDegrees = Math.abs(getShortestDistance(expectedEnc2Deg, deg2));

            if (errorDegrees < minError) {
                minError = errorDegrees;
                bestMatchAngle = candidateTurretDeg;
            }
        }

        // If even the best candidate exceeds the match threshold, both encoders disagree —
        // likely a wiring fault or encoder failure. Return NaN so the caller can react safely.
        double thresholdDegrees = MATCH_THRESHOLD * 360.0;
        if (minError > thresholdDegrees) {
            System.err.println("CRITICAL TURRET ERROR: CRT Solver mismatch. Error: " + minError);
            return Double.NaN;
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
        turretRealTarget = degrees;
        double clampedDegrees = Math.max(TurretConstants.TURRET_MIN_ROTATION, Math.min(TurretConstants.TURRET_MAX_ROTATION, degrees));
        
        // Convert to mechanism rotations, then multiply by gear ratio for NATIVE Motor Rotations
        double targetMechanismRotations = clampedDegrees / 360.0;
        turretPIDTarget = targetMechanismRotations * TURRET_GEAR_RATIO;
        turretMotor.setControl(turretControl.withPosition(turretPIDTarget));
    }

    @Override
    // 0-1.4 from minimum elevation to maximum elevation
    public void setHoodAngle(double hoodElevation) {
        if ( hoodElevation <  TurretConstants.HOOD_MIN_ANGLE) {
            hoodElevation = TurretConstants.HOOD_MIN_ANGLE;
        } else if ( hoodElevation > TurretConstants.HOOD_MAX_ANGLE) {
            hoodElevation = TurretConstants.HOOD_MAX_ANGLE;
        }

        hoodElevationTarget = hoodElevation * TurretConstants.HOOD_GEAR_RATIO;
        hoodPID.setSetpoint(hoodElevationTarget, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void setFlywheelVelocity(double mainVelocity, double upperVelocity) {
        targetFlywheelVelocity = mainVelocity;
        targetUpperFlywheelVelocity = upperVelocity;
        flywheelMotor.setControl(flywheelControl.withVelocity(targetFlywheelVelocity));
        upperFlywheelMotor.setControl(upperFlywheelControl.withVelocity(targetUpperFlywheelVelocity));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(allSignals);
        
        double actualAngle = calculateAbsolutePositionCRT();
        inputs.turretAngleDegrees = actualAngle;
        if(!Double.isNaN(actualAngle)){
            inputs.turretAngle = Rotation2d.fromDegrees(actualAngle);
        }

        inputs.turretPIDSetpoint = turretPIDTarget;
        inputs.turretRealTarget = turretRealTarget;
        inputs.turretMotorPosition = turretMotorPositionSignal.getValueAsDouble();
        inputs.turretMotorCurrent = turretMotorCurrentSignal.getValueAsDouble();

        inputs.hoodMotorCurrent = hoodMotor.getOutputCurrent();
        inputs.hoodEncoderPosition = hoodEncoderPositionSignal.getValueAsDouble();
        inputs.hoodMotorPIDTarget = hoodElevationTarget;
        inputs.hoodMotorPosition = hoodMotor.getEncoder().getPosition();

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

        // Log the actual error (Measured - Target)
        // Positive values = Wheel is spinning faster than target (Overshooting)
        // Negative values = Wheel is still spinning up (Undershooting)
        inputs.flywheelVelocityError = inputs.flywheelMotorVelocity - targetFlywheelVelocity;
        inputs.upperFlywheelVelocityError = inputs.upperFlywheelMotorVelocity - targetUpperFlywheelVelocity;

        inputs.areFlywheelsAtVelocity = areFlywheelsAtVelocity();
        inputs.isTurretAtTarget = isTurretAtTarget();
    }

    private boolean areFlywheelsAtVelocity() {
        final double flywheelV = flywheelMotorVelocitySignal.getValueAsDouble();
        final double flywheelT = targetFlywheelVelocity;

        final double upperFlywheelV = upperFlywheelMotorVelocitySignal.getValueAsDouble();
        final double upperFlywheelT = targetUpperFlywheelVelocity; 

        return (targetFlywheelVelocity > 0 &&
            (Math.abs(flywheelV - flywheelT) < (0.30 * targetFlywheelVelocity)) && ((targetUpperFlywheelVelocity > 0)
            && (Math.abs(upperFlywheelV - upperFlywheelT) < (0.30 * targetUpperFlywheelVelocity))));
    }

    private boolean isTurretAtTarget() {
        final double position = turretMotorPositionSignal.getValueAsDouble();
        final double error = Math.abs(turretPIDTarget - position);
        final double realTarget = ((turretRealTarget / 360) * TURRET_GEAR_RATIO);
        return (Math.abs(turretPIDTarget - realTarget) < 1)  && (error < ALLOWED_TURRET_ERROR_ROTATIONS);
    }
}