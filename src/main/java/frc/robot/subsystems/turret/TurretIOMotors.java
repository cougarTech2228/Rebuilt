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
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

    private double turretPIDTarget;
    private double hoodElevationTarget; //0 .. 100
    private double hoodAngleTarget; //0 .. 0.9

    private final CANcoder enc31; // The encoder on the 31T gear
    private final CANcoder enc37; // The encoder on the 37T gear
    private final CANcoder encHood;

    private final SparkMax turretMotor;
    private final SparkClosedLoopController turretPID;

    private final TalonFXS hoodMotor;
    private final MotionMagicVoltage hoodControl;

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
    private static final double RATIO_31 = MAIN_TEETH_ENCODER / TEETH_31; // Exactly 5.0
    private static final double RATIO_37 = MAIN_TEETH_ENCODER / TEETH_37; // ~4.189

    // Turret Motor Gear Ratio (10:1 planetary and 125/18 pinion) -> ~69.444
    private static final double TURRET_GEAR_RATIO = (MAIN_TEETH_BELT / 18.0) * 10.0;

    // How close the two encoders must match to be considered valid (in rotations)
    private static final double MATCH_THRESHOLD = 0.05;

    // degrees of error allowed for turret PID aiming
    private static final double ALLOWED_TURRET_ERROR = 2.0;

    // Status signals for low-latency reading
    private final StatusSignal<Angle> pos31Signal;
    private final StatusSignal<Angle> pos37Signal;

    private final StatusSignal<AngularVelocity> hoodMotorVelocitySignal;
    private final StatusSignal<Voltage> hoodMotorVoltageSignal;
    private final StatusSignal<Current> hoodMotorCurrentSignal;
    private final StatusSignal<Angle> hoodEncoderPositionSignal;
    private final StatusSignal<Angle> hoodMotorPositionSignal;

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

        turretConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        turretConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.2)
            .i(0)
            .d(0);
        turretConfig.closedLoop.maxMotion.maxAcceleration(7000);
        turretConfig.closedLoop.maxMotion.cruiseVelocity(3500);
        turretConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        turretConfig.closedLoop.maxMotion.allowedProfileError(1);

        // NATIVE UNITS: kV is DutyCycle output (0-1) per RPM. 
        // NEO free speed is ~5676 RPM at 1.0 DutyCycle. So kV = 1.0 / 5676 = 0.00017
        turretConfig.closedLoop.feedForward
            .kS(0.0) 
            .kV(0.00017) 
            .kA(0.0);

        // Set limits in Native Motor Rotations
        turretConfig.softLimit
            .forwardSoftLimit((float)((TurretConstants.TURRET_MAX_ROTATION / 360.0) * TURRET_GEAR_RATIO))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit((float)((TurretConstants.TURRET_MIN_ROTATION / 360.0) * TURRET_GEAR_RATIO))
            .reverseSoftLimitEnabled(true);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMotor = new TalonFXS(frc.robot.Constants.CAN_ID_TURRET_HOOD_MOTOR, frc.robot.RobotContainer.kRio);
        hoodControl = new MotionMagicVoltage(0);

        flywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_MOTOR_FLYWHEEL, frc.robot.RobotContainer.kRio);
        flywheelControl = new MotionMagicVelocityVoltage(0);

        upperFlywheelMotor = new TalonFX(frc.robot.Constants.CAN_ID_UPPER_FLYWHEEL_MOTOR, frc.robot.RobotContainer.kRio);
        upperFlywheelControl = new MotionMagicVelocityVoltage(0);

        enc31 = new CANcoder(CAN_ID_TURRET_ENCODER_31T, frc.robot.RobotContainer.kRio);
        enc37 = new CANcoder(CAN_ID_TURRET_ENCODER_37T, frc.robot.RobotContainer.kRio);

        CANcoderConfiguration config31 = new CANcoderConfiguration();
        config31.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config31.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config31.MagnetSensor.MagnetOffset = 0.089111;
        enc31.getConfigurator().apply(config31);

        CANcoderConfiguration config37 = new CANcoderConfiguration();
        config37.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config37.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config37.MagnetSensor.MagnetOffset = 0.443115;
        enc37.getConfigurator().apply(config37);

        pos31Signal = enc31.getAbsolutePosition();
        pos37Signal = enc37.getAbsolutePosition();

        BaseStatusSignal.refreshAll(pos31Signal, pos37Signal);

        seedTurretPosition();

        TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
        hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        hoodConfig.Slot0.kP = 20.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kV = 0.0;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 5.0; // hood encoder rotations/sec
        hoodConfig.MotionMagic.MotionMagicAcceleration = 20.0; // hood encoder rotations/sec^2
        hoodConfig.MotionMagic.MotionMagicJerk = 0.0;

        
        hoodConfig.ExternalFeedback.FeedbackRemoteSensorID = CAN_ID_TURRET_HOOD_ENCODER;
        hoodConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        hoodConfig.ExternalFeedback.RotorToSensorRatio = 50.0;

        hoodConfig.ClosedLoopGeneral.ContinuousWrap = false;

        // Set limits to safe values
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.HOOD_MAX_ANGLE;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.HOOD_MIN_ANGLE;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodMotor.getConfigurator().apply(hoodConfig);

        encHood = new CANcoder(CAN_ID_TURRET_HOOD_ENCODER, frc.robot.RobotContainer.kRio);
        CANcoderConfiguration configHood = new CANcoderConfiguration();
        configHood.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        configHood.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        configHood.MagnetSensor.MagnetOffset = 0.290039;
        encHood.getConfigurator().apply(configHood);

        hoodMotorVelocitySignal = hoodMotor.getVelocity();
        hoodMotorVoltageSignal = hoodMotor.getMotorVoltage();
        hoodMotorCurrentSignal = hoodMotor.getStatorCurrent();
        hoodMotorPositionSignal = hoodMotor.getPosition();
        hoodEncoderPositionSignal = encHood.getPosition();

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        TalonFXConfiguration upperFlywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = 0.5;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;
        flywheelConfig.Slot0.kV = 0.1;
        flywheelConfig.Slot0.kA = 0.0;

        flywheelConfig.CurrentLimits.StatorCurrentLimit = 120.0; // Amps
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelControl.Acceleration = 200.0;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotorVelocitySignal = flywheelMotor.getVelocity();
        flywheelMotorVoltageSignal = flywheelMotor.getMotorVoltage();
        flywheelMotorCurrentSignal = flywheelMotor.getStatorCurrent();

        upperFlywheelConfig.Slot0.kP = 0.5;
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
    }

    /**
     * Seeds the internal NEO encoder using the 31T and 37T encoders.
     */
    public void seedTurretPosition() {
        double absoluteTurretDegrees = calculateAbsolutePositionCRT();

        // Convert degrees to mechanism rotations (0.5 = 180 deg)
        double turretRotations = absoluteTurretDegrees / 360.0;

        // Seed the SparkMax in NATIVE Motor Rotations
        turretMotor.getEncoder().setPosition(turretRotations * TURRET_GEAR_RATIO);
        
        System.out.println("Turret Seeded at: " + absoluteTurretDegrees + " degrees");
    }

    /**
     * Solves for Absolute Position using CRT.
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

        // Try each plausible number of full enc31 wraps. Each increment of k adds ~89° of
        // implied turret travel (360° / RATIO_31). Range -2 to 4 covers -26° to 248° with margin.
        for (int k = -5; k <= 5; k++) {
            // Unwrap enc31 by k full rotations, then back-calculate the implied turret angle.
            double candidateTurretDeg = (deg1 + (k * 360.0)) / RATIO_31;

            // Discard candidates outside the physical travel range.
            if (candidateTurretDeg < TurretConstants.TURRET_MIN_ROTATION ||
                candidateTurretDeg > TurretConstants.TURRET_MAX_ROTATION) {
                continue;
            }

            // If the turret is really at candidateTurretDeg, predict what enc37 should read.
            double expectedEnc2Deg = normalizeDegrees(candidateTurretDeg * RATIO_37);

            // Score this candidate by how closely enc37's actual reading matches the prediction.
            // The correct k will produce near-zero error; wrong k values will be off by tens of degrees.
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
        double clampedDegrees = Math.max(TurretConstants.TURRET_MIN_ROTATION, Math.min(TurretConstants.TURRET_MAX_ROTATION, degrees));
        
        // Convert to mechanism rotations, then multiply by gear ratio for NATIVE Motor Rotations
        double targetMechanismRotations = clampedDegrees / 360.0;
        double newTarget = targetMechanismRotations * TURRET_GEAR_RATIO;
        
        if (Math.abs(newTarget - turretPIDTarget) > 0.01) {
            turretPIDTarget = newTarget;
            turretPID.setSetpoint(turretPIDTarget, ControlType.kMAXMotionPositionControl);
        }
    }

    @Override
    // 0-1.4 from minimum elevation to maximum elevation
    public void setHoodAngle(double hoodElevation) {
        if ( hoodElevation <  TurretConstants.HOOD_MIN_ANGLE) {
            hoodElevation = TurretConstants.HOOD_MIN_ANGLE;
        } else if ( hoodElevation > TurretConstants.HOOD_MAX_ANGLE) {
            hoodElevation = TurretConstants.HOOD_MAX_ANGLE;
        }

        hoodElevationTarget = hoodElevation; // Elevation 0 .. 1.4
        hoodMotor.setControl(hoodControl.withPosition(hoodElevationTarget));
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
        BaseStatusSignal.refreshAll(
            pos31Signal,
            pos37Signal,
            hoodMotorCurrentSignal,
            hoodMotorVelocitySignal,
            hoodMotorVoltageSignal,
            hoodEncoderPositionSignal,
            hoodMotorPositionSignal,
            flywheelMotorVelocitySignal,
            upperFlywheelMotorVelocitySignal,
            upperFlywheelMotorVoltageSignal,
            upperFlywheelMotorCurrentSignal
        );

        double actualAngle = calculateAbsolutePositionCRT();
        if(!Double.isNaN(actualAngle)){
            inputs.turretAngle = Rotation2d.fromDegrees(actualAngle);
        }

        inputs.turretPIDSetpoint = turretPIDTarget;
        inputs.turretMotorPosition = turretMotor.getEncoder().getPosition();

        inputs.hoodTargetElevationPercent = hoodElevationTarget;
        inputs.hoodMotorVelocity = hoodMotorVelocitySignal.getValueAsDouble();
        inputs.hoodMotorVoltage = hoodMotorVoltageSignal.getValueAsDouble();
        inputs.hoodMotorCurrent = hoodMotorCurrentSignal.getValueAsDouble();
        inputs.hoodEncoderPosition = hoodEncoderPositionSignal.getValueAsDouble();
        inputs.hoodPIDTargetAngle = hoodAngleTarget;
        inputs.hoodMotorPosition = hoodMotorPositionSignal.getValueAsDouble();

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

        inputs.areFlywheelsAtVelocity = areFlywheelsAtVelocity();
        inputs.isTurretAtTarget = isTurretAtTarget();
    }

    private boolean areFlywheelsAtVelocity() {
        final double flywheelV = flywheelMotorVelocitySignal.getValueAsDouble();
        final double flywheelT = targetFlywheelVelocity;

        final double upperFlywheelV = upperFlywheelMotorVelocitySignal.getValueAsDouble();
        final double upperFlywheelT = targetUpperFlywheelVelocity; 

        return (targetFlywheelVelocity > 0 &&
            (Math.abs(flywheelV - flywheelT) < (0.05 * targetFlywheelVelocity)) && ((targetUpperFlywheelVelocity > 0) 
            && (Math.abs(upperFlywheelV - upperFlywheelT) < (0.05 * targetUpperFlywheelVelocity))));
    }

    private boolean isTurretAtTarget() {
        // return true;
        // Calculate error in mechanism rotations
        // double currentMechanismRotations = turretMotor.getEncoder().getPosition() / TURRET_GEAR_RATIO;
        // return Math.abs((currentMechanismRotations * 360.0) - turretAngleTarget) <= ALLOWED_TURRET_ERROR;
        return turretPID.isAtSetpoint();
    }
}