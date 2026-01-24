package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class TurretIOTalonFX implements TurretIO {

    private double turretAngleTarget;

    private final CANcoder enc19; // The encoder on the 19T gear
    private final CANcoder enc21; // The encoder on the 21T gear

    private final TalonFX turretMotor;
    private final MotionMagicVoltage turretControl;

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

    public TurretIOTalonFX() {
        turretMotor = new TalonFX(frc.robot.Constants.CAN_ID_TURRET_MOTOR);
        turretControl = new MotionMagicVoltage(0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 300.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 1.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 2.0; // turret roations/sec
        config.MotionMagic.MotionMagicAcceleration = 10.0; // turret roations/sec^2
        config.MotionMagic.MotionMagicJerk = 0.0;

        // Motor Revs per 1 Turret Rev = 5 * (200/18) = 55.555...
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 55.55555555555556;
        config.ClosedLoopGeneral.ContinuousWrap = false;
        // Set limits to +/- 190 degrees to allow full 360 coverage with overlap
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.53;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.53;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        turretMotor.getConfigurator().apply(config);

        enc19 = new CANcoder(19);
        enc21 = new CANcoder(21);

        CANcoderConfiguration config19 = new CANcoderConfiguration();
        config19.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config19.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config19.MagnetSensor.MagnetOffset = 0.047607;// 0.318848; // Set 19T Zero Offset
        enc19.getConfigurator().apply(config19);

        CANcoderConfiguration config21 = new CANcoderConfiguration();
        config21.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config21.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config21.MagnetSensor.MagnetOffset = -0.249756; // -0.493896; // Set 21T Zero Offset
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
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(pos19Signal, pos21Signal, turretMotorPositionSignal);

        double actualAngle = getTurretPositionDegrees();
        inputs.turretAngle = Rotation2d.fromDegrees(actualAngle);
        inputs.turretPIDTargetAngle = turretAngleTarget;
        inputs.turretPIDActualAngle = actualAngle;
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
            // Hypothesis: k turns + partial reading
            double candidateTurretRotations = (k + p19) / RATIO_19;

            // Prediction: What should 21T read?
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
            return degrees;
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

}
