package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {

    private static final double TURRET_MOTOR_GEARBOX_RATIO = 10.0;
    private static final double TURRET_GEAR_RATIO = (125.0 / 18.0) * TURRET_MOTOR_GEARBOX_RATIO;
    private static final double HOOD_GEAR_RATIO = 50.0;

    private final DCMotorSim turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.01, TURRET_GEAR_RATIO),
        DCMotor.getNeo550(1)
    );

    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeo550(1), 0.01, HOOD_GEAR_RATIO),
        DCMotor.getNeo550(1),
        HOOD_GEAR_RATIO,
        0.2,
        0.0,
        TurretConstants.HOOD_MAX_ANGLE * (2 * Math.PI),
        true,
        0.0
    );

    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), 0.005, 1.0),
        DCMotor.getKrakenX44(1)
    );
    
    private final FlywheelSim upperFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), 0.005, 1.0),
        DCMotor.getKrakenX44(1)
    );

    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(0.0, 0.065);
    private final ProfiledPIDController turretPID = new ProfiledPIDController(
        2.0, 0, 0, 
        new TrapezoidProfile.Constraints(180.0, 800.0)
    );

    private final ProfiledPIDController hoodPID = new ProfiledPIDController(
        10.0, 0, 0, 
        new TrapezoidProfile.Constraints(5.0, 20.0)
    );

    private final SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0.0, 0.10);
    private final PIDController flywheelPID = new PIDController(0.2, 0, 0);

    private final SimpleMotorFeedforward upperFlywheelFF = new SimpleMotorFeedforward(0.0, 0.10);
    private final PIDController upperFlywheelPID = new PIDController(0.2, 0, 0);

    private double turretRealTargetDegrees = 0.0;
    private double turretMotorTargetRotations = 0.0;
    
    private double hoodElevationTarget = 0.0;
    
    private double targetFlywheelRPS = 0.0;
    private double targetUpperFlywheelRPS = 0.0;

    public TurretIOSim() {
        turretPID.reset(0);
        hoodPID.reset(0);
    }

    @Override
    public void setTurretAngle(double degrees) {
        this.turretRealTargetDegrees = MathUtil.clamp(degrees, TurretConstants.TURRET_MIN_ROTATION, TurretConstants.TURRET_MAX_ROTATION);
        this.turretMotorTargetRotations = (this.turretRealTargetDegrees / 360.0) * TURRET_GEAR_RATIO;
    }

    @Override
    public void setHoodAngle(double hoodAngle) {
        this.hoodElevationTarget = MathUtil.clamp(hoodAngle, TurretConstants.HOOD_MIN_ANGLE, TurretConstants.HOOD_MAX_ANGLE);
    }

    @Override
    public void setFlywheelVelocity(double mainVelocity, double upperVelocity) {
        this.targetFlywheelRPS = mainVelocity;
        this.targetUpperFlywheelRPS = upperVelocity;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turretSim.update(0.020);
        hoodSim.update(0.020);
        flywheelSim.update(0.020);
        upperFlywheelSim.update(0.020);

        // Turret Update
        double currentTurretOutputRotations = turretSim.getAngularPositionRotations();
        double currentTurretMotorRotations = currentTurretOutputRotations * TURRET_GEAR_RATIO;
        
        double turretPidVolts = turretPID.calculate(currentTurretMotorRotations, turretMotorTargetRotations);
        double turretFfVolts = turretFF.calculate(turretPID.getSetpoint().velocity);
        
        turretSim.setInput(MathUtil.clamp(turretPidVolts + turretFfVolts, -12.0, 12.0));
        
        double currentTurretDegrees = currentTurretOutputRotations * 360.0;

        inputs.turretPIDSetpoint = turretMotorTargetRotations;
        inputs.turretRealTarget = turretRealTargetDegrees;
        inputs.turretMotorPosition = currentTurretMotorRotations;
        inputs.turretMotorCurrent = turretSim.getCurrentDrawAmps();
        inputs.turretAngle = Rotation2d.fromDegrees(currentTurretDegrees);

        inputs.enc31t = ((currentTurretDegrees / 360.0) * (160.0 / 31.0)) % 1.0;
        if (inputs.enc31t < 0) inputs.enc31t += 1.0;
        
        inputs.enc37t = ((currentTurretDegrees / 360.0) * (160.0 / 37.0)) % 1.0;
        if (inputs.enc37t < 0) inputs.enc37t += 1.0;

        // Hood Update
        double currentHoodEncoderRotations = hoodSim.getAngleRads() / (2 * Math.PI);
        double hoodVolts = MathUtil.clamp(
            hoodPID.calculate(currentHoodEncoderRotations, hoodElevationTarget),
            -12.0, 12.0
        );
        hoodSim.setInput(hoodVolts);

        // inputs.hoodTargetElevationPercent = hoodElevationTarget;
        // inputs.hoodPIDTargetAngle = hoodElevationTarget;
        inputs.hoodEncoderPosition = currentHoodEncoderRotations;
        inputs.hoodMotorPosition = currentHoodEncoderRotations * HOOD_GEAR_RATIO;
        inputs.hoodMotorCurrent = hoodSim.getCurrentDrawAmps();

        // Flywheel Update
        double currentFlywheelRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
        double flywheelVolts = MathUtil.clamp(
            flywheelFF.calculate(targetFlywheelRPS) + flywheelPID.calculate(currentFlywheelRPS, targetFlywheelRPS),
            -12.0, 12.0
        );
        flywheelSim.setInput(flywheelVolts);

        double currentUpperFlywheelRPS = upperFlywheelSim.getAngularVelocityRPM() / 60.0;
        double upperFlywheelVolts = MathUtil.clamp(
            upperFlywheelFF.calculate(targetUpperFlywheelRPS) + upperFlywheelPID.calculate(currentUpperFlywheelRPS, targetUpperFlywheelRPS),
            -12.0, 12.0
        );
        upperFlywheelSim.setInput(upperFlywheelVolts);

        inputs.flywheelMotorVelocity = currentFlywheelRPS;
        inputs.flywheelMotorVoltage = flywheelVolts;
        inputs.flywheelMotorCurrent = flywheelSim.getCurrentDrawAmps();
        inputs.flywheelPIDTargetVelocity = targetFlywheelRPS;

        inputs.upperFlywheelMotorVelocity = currentUpperFlywheelRPS;
        inputs.upperFlywheelMotorVoltage = upperFlywheelVolts;
        inputs.upperFlywheelMotorCurrent = upperFlywheelSim.getCurrentDrawAmps();
        inputs.upperFlywheelPIDTargetVelocity = targetUpperFlywheelRPS;

        inputs.areFlywheelsAtVelocity = areFlywheelsAtVelocity(currentFlywheelRPS, currentUpperFlywheelRPS);
        inputs.isTurretAtTarget = isTurretAtTarget(currentTurretMotorRotations);
    }

    private boolean areFlywheelsAtVelocity(double currentMainRPS, double currentUpperRPS) {
        return (targetFlywheelRPS > 0 &&
            (Math.abs(currentMainRPS - targetFlywheelRPS) < (0.05 * targetFlywheelRPS)) && 
            ((targetUpperFlywheelRPS > 0) && (Math.abs(currentUpperRPS - targetUpperFlywheelRPS) < (0.05 * targetUpperFlywheelRPS))));
    }

    private boolean isTurretAtTarget(double currentMotorPosition) {
        double allowedErrorRotations = (2.0 * ((125.0 / 18.0) * 10.0)) / 360.0; 
        return (turretMotorTargetRotations != 0.0 || turretRealTargetDegrees == 0.0) && 
               (Math.abs(turretMotorTargetRotations - currentMotorPosition) < allowedErrorRotations);
    }
}