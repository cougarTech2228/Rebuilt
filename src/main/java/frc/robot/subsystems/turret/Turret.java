package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase{
    public enum TurretAimTarget  {
        Hub,
        LobLower,
        LobUpper
    };

    private TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

    private Drive driveSubsystem;

    private TurretAimTarget aimTarget = TurretAimTarget.Hub;
    private boolean climbModeEnabled = false;

    // Store the compensated target each loop
    private Pose2d virtualTargetPose = new Pose2d();
    private double shotStability = 1.0; // 0 = stable, 1 = unstable calculations    
    private static final double MAX_SHOT_STABILITY = 0.7;

    private double currentHoodAngle = 0;

    private LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private SlewRateLimiter slewDistanceFilter = new SlewRateLimiter(2);

    private boolean turretEnabled = false;

    private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flightMap = new InterpolatingDoubleTreeMap();

    public Turret(TurretIO turretIO, Drive driveSubsystem) {
        this.turretIO = turretIO;
        this.driveSubsystem = driveSubsystem;

        SmartDashboard.putNumber("TurretHoodElevation", 0.0);
        SmartDashboard.putNumber("TurretFlywheelVelocity", 0.0);
        SmartDashboard.putNumber("TurretTestDistance", 0.0);
        SmartDashboard.putNumber("TurretTestFlywheelRatio", 1.0);

        SmartDashboard.putNumber("TurretAngle", 0.0);
        SmartDashboard.putNumber("TurretTestX", 4.6129);
        SmartDashboard.putNumber("TurretTestIntercept", TurretConstants.turretTestIntercept);
        SmartDashboard.putNumber("TurretTestAngleIntercept", 0.7);

        SmartDashboard.setPersistent("TurretTestX");
        SmartDashboard.setPersistent("TurretTestIntercept");
        SmartDashboard.setPersistent("TurretTestAngleIntercept");

        // Shoot-on-the-move tuning parameter. Represents the average horizontal m/s of the fuel.
        SmartDashboard.putNumber("TurretShotSpeedMpS", 15.0);

        // map references, (distance away from hub, hood angle)
        hoodAngleMap.put(1.000, 0.0);
        hoodAngleMap.put(1.500, 0.1);
        hoodAngleMap.put(2.000, 0.2);
        hoodAngleMap.put(2.500, 0.3);
        hoodAngleMap.put(3.000, 0.4);
        hoodAngleMap.put(3.500, 0.5);
        hoodAngleMap.put(4.000, 0.6);
        hoodAngleMap.put(4.500, 0.7);

        // map references, (distance away from the hub, flywheel velocity)
        flywheelVelocityMap.put(1.000, 35.678);
        flywheelVelocityMap.put(1.500, 36.463);
        flywheelVelocityMap.put(2.000, 38.157);
        flywheelVelocityMap.put(2.500, 40.468);
        flywheelVelocityMap.put(3.000, 43.106);
        flywheelVelocityMap.put(3.500, 45.779);
        flywheelVelocityMap.put(4.000, 48.196);
        flywheelVelocityMap.put(4.500, 50.065);
        flywheelVelocityMap.put(5.000, 51.095);

        // map references, (distance away from hub, seconds airtime)
        flightMap.put(1.500, 0.939);
        flightMap.put(2.000, 1.045);
        flightMap.put(2.500, 1.119);
        flightMap.put(3.000, 1.254);
        flightMap.put(3.500, 1.341);
        flightMap.put(4.000, 1.518);

    }

    private double getTargetDistance() {
        Pose2d robotPose = driveSubsystem.getPose();
        
        // Return distance to the virtual compensated target from the physical turret offset
        return robotPose.getTranslation().plus(
            TurretConstants.TurretOffset.getTranslation().rotateBy(robotPose.getRotation()))
            .getDistance(virtualTargetPose.getTranslation());
    }

    /**
     * Calculates the estimated time of flight for the game piece to reach the target.
     */
    private double estimateTimeOfFlight(double distance) {

        // // Basic Average (not currently active)
        // // double averageHorizontalSpeed = SmartDashboard.getNumber("TurretShotSpeedMpS", 15.0);

        // // = -0.1343x2 + 1.163x + 0.1467
        // double averageHorizontalSpeed = (-0.1343 * distance * distance) + (1.163 * distance) + 0.1467;
        // double timeOfFlightSeconds = distance / averageHorizontalSpeed;

        // // sanity check
        // return MathUtil.clamp(timeOfFlightSeconds, 0.1, 2.0);

        // Use the measured flight-time look-up table, not a derived formula.
        // The table implicitly encodes the physics — this is the key advantage
        // of the ToF-recursion approach per the docs.
        return MathUtil.clamp(flightMap.get(distance), 0.1, 2.0);

    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);


        // --- Shoot On The Move Math ---
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d realTargetPose = getTargetPoint(aimTarget);
        

        // 1. Get the robot's velocity
        ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
        Translation2d robotVelocityRR = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        // 2. Add the "whip" velocity of the turret swinging around the robot's center of rotation
        Translation2d turretOffset = TurretConstants.TurretOffset.getTranslation();
        Translation2d spinVelocityRR = new Translation2d(
            -speeds.omegaRadiansPerSecond * turretOffset.getY(),
            speeds.omegaRadiansPerSecond * turretOffset.getX()
        );
        Translation2d totalTurretVelocityRR = robotVelocityRR.plus(spinVelocityRR);

        // 3. Convert the true physical turret velocity to Field Relative
        Translation2d fieldRelativeVelocity = totalTurretVelocityRR.rotateBy(robotPose.getRotation());

        // // 4. Iterative Solver for Time of Flight (ToF)
        // Translation2d virtualTargetTrans = realTargetPose.getTranslation();
        // Translation2d absoluteTurretPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        // for (int i = 0; i < 3; i++) {
        //     // Distance from the physical turret to the virtual target
        //     double virtualDistance = absoluteTurretPosition.getDistance(virtualTargetTrans);
        //     double timeOfFlight = estimateTimeOfFlight(virtualDistance);

        //     // Offset the target point by the velocity we will impart during the time of flight
        //     Translation2d movingOffset = fieldRelativeVelocity.times(timeOfFlight);
        //     virtualTargetTrans = realTargetPose.getTranslation().minus(movingOffset);
        // }

        // virtualTargetPose = new Pose2d(virtualTargetTrans, realTargetPose.getRotation());

        Translation2d virtualTargetTrans = realTargetPose.getTranslation();
        Translation2d absoluteTurretPosition = robotPose.getTranslation()
            .plus(turretOffset.rotateBy(robotPose.getRotation()));

        // Track the last two ToF values to compute the contraction rate (stability)
        // |φ'| = |τ_n - τ_{n-1}| / |τ_{n-1} - τ_{n-2}|
        double tofPrev2 = 0.0;
        double tofPrev1 = 0.0;
        double tofCurrent = 0.0;
        shotStability = 1.0; // default to fragile until we compute it

        // 3–5 iterations is enough per the docs; 5 gives a wider convergence envelope
        // with diminishing returns beyond that
        for (int i = 0; i < 5; i++) {
            double virtualDistance = absoluteTurretPosition.getDistance(virtualTargetTrans);
            tofCurrent = estimateTimeOfFlight(virtualDistance); // now reads from flight map

            Translation2d movingOffset = fieldRelativeVelocity.times(tofCurrent);
            virtualTargetTrans = realTargetPose.getTranslation().minus(movingOffset);

            // Compute contraction rate once we have three ToF samples
            // Avoid divide-by-zero only use terms larger than machine epsilon
            if (i >= 2) {
                double deltaN = Math.abs(tofCurrent - tofPrev1);
                double deltaNm1 = Math.abs(tofPrev1 - tofPrev2);
                if (deltaNm1 > 1e-9) {
                    shotStability = deltaN / deltaNm1;
                }
            }

            tofPrev2 = tofPrev1;
            tofPrev1 = tofCurrent;
        }

        virtualTargetPose = new Pose2d(virtualTargetTrans, realTargetPose.getRotation());

        if (turretEnabled) {
            if (SmartDashboard.getBoolean("TestMode", false)) {
                setHoodElevation(SmartDashboard.getNumber("TurretHoodElevation", 0.0));
                double ratio = SmartDashboard.getNumber("TurretTestFlywheelRatio", 1.0);
                double velocity = SmartDashboard.getNumber("TurretFlywheelVelocity", 0.0);
                setFlywheelVelocity(velocity, ratio * velocity);

                setAimTarget(SmartDashboard.getNumber("TurretAngle", 0.0));
            } else {
                double dist = turretInputs.filteredTargetDistance;
                double vel = getVelocityForTarget(dist, false);
                setFlywheelVelocity(vel, vel * 1.2);

                double angle = getAngleForTarget(dist);
                if (Math.abs(angle - currentHoodAngle) > 0.01) {
                    currentHoodAngle = angle;
                    setHoodElevation(angle);
                }
            }
        } else {
            setFlywheelVelocity(0, 0);
            // no need to move the hood on disable
        }

        turretInputs.targetDistance = getTargetDistance();
        turretInputs.filteredTargetDistance = distanceFilter.calculate(turretInputs.targetDistance);
        turretInputs.slewFilteredTargetDistance = slewDistanceFilter.calculate(turretInputs.filteredTargetDistance);

        // Log the real target vs the virtual aim target
        Logger.recordOutput("Turret/RealTargetPose", realTargetPose);
        Logger.recordOutput("Turret/VirtualTargetPose", virtualTargetPose);
        Logger.recordOutput("Turret/ShotStability", shotStability); 
        Logger.processInputs("Turret", turretInputs);

        RobotContainer.turretPose = new Pose3d(
            RobotContainer.turretPose.getX(),
            RobotContainer.turretPose.getY(),
            RobotContainer.turretPose.getZ(),
            new Rotation3d(turretInputs.turretAngle));
        
        if (climbModeEnabled) {
            turretIO.setTurretAngle(90);
        }
    }

    public void setAimTarget(double turretAngle) {
        turretIO.setTurretAngle(turretAngle);
    }

    public void setAimTarget(TurretAimTarget target) {
        if (climbModeEnabled) {
            turretIO.setTurretAngle(180);
            return;
        }
        aimTarget = target;
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d targetPose = virtualTargetPose;

        turretInputs.turretTargetPoint = targetPose;

        Translation2d turretXY = robotPose.getTranslation().plus(
            TurretConstants.TurretOffset.getTranslation().rotateBy(robotPose.getRotation()));

        Translation2d targetXY = targetPose.getTranslation();

        // Calculate the difference vector (Vector from Robot -> Target)
        Translation2d difference = targetXY.minus(turretXY);

        // Get the angle of that vector relative to the field
        Rotation2d angleToTargetFieldRelative = difference.getAngle();

        // Subtract robot's rotation to make it robot-relative
        Rotation2d angleToTargetRobotRelative = angleToTargetFieldRelative.minus(robotPose.getRotation());

        // Wrap the angle so it maps exactly to the physical 360-degree window of the mechanism.
        double turretAngle = MathUtil.inputModulus(
            angleToTargetRobotRelative.getDegrees(),
            TurretConstants.TURRET_MIN_ROTATION,
            TurretConstants.TURRET_MIN_ROTATION + 360.0
        );

        // Because we wrapped it to the physical minimum, anything greater than the MAX
        // is mathematically sitting inside the deadzone/keep-out.
        turretInputs.isTargetInKeepOut = turretAngle > TurretConstants.TURRET_MAX_ROTATION;

        if (turretInputs.isTargetInKeepOut) {
            // It's in the keep-out zone. Which valid boundary is physically closer?
            double distToMax = Math.abs(turretAngle - TurretConstants.TURRET_MAX_ROTATION);

            // The distance to the minimum involves wrapping against the upper bound of our Modulus
            double distToMin = Math.abs((TurretConstants.TURRET_MIN_ROTATION + 360.0) - turretAngle);

            turretAngle = (distToMax < distToMin)
                ? TurretConstants.TURRET_MAX_ROTATION
                : TurretConstants.TURRET_MIN_ROTATION;
        }

        if (SmartDashboard.getBoolean("TestMode", false)) {
            turretIO.setTurretAngle(SmartDashboard.getNumber("TurretAngle", 0.0));
        } else {
            turretIO.setTurretAngle(turretAngle);
        }
    }

    public void setHoodElevation(double elevation) {
        turretIO.setHoodAngle(elevation);
    }

    private double getAngleForTarget(double distance) {
        double angle;

        if (aimTarget == TurretAimTarget.Hub) {
            // y = 0.1933x - 0.1842 
            // angle = (0.1933 * distance) - 0.1842;
            angle = hoodAngleMap.get(distance);
        } else {
            angle = 1.3;
        }

        // sanity check the values
        angle = Math.min(angle, TurretConstants.HOOD_MAX_ANGLE);
        angle = Math.max(angle, TurretConstants.HOOD_MIN_ANGLE);

        Logger.recordOutput("Turret/RealAngle", angle);
        return angle;
    }

    private double getVelocityForTarget(double distance, boolean test) {
        double velocity;

        if (test) {
            distance = SmartDashboard.getNumber("TurretTestDistance", 0.0);
        }

        if (aimTarget == TurretAimTarget.Hub) {
            // y = -0.3884x3 + 3.5657x2 - 5.4995x + 37.699
            // velocity = (-0.3884 * distance * distance * distance)
            //     + (3.5657 * distance * distance)
            //     - (5.4995 * distance)
            //     + 38;
            velocity = flywheelVelocityMap.get(distance);
        } else {
            //y = 5.2631x + 18.264
            velocity = (5.2631 * distance) + 18.264;
        }

        // cap the speed to an achievable value
        velocity = Math.min(velocity, TurretConstants.MAX_FLYWHEEL_SPEED);

        // ensure we don't end up with a negative value or super slow speed
        velocity = Math.max(velocity, TurretConstants.MIN_FLYWHEEL_SPEED);
        Logger.recordOutput("Turret/realDistance", distance);
        Logger.recordOutput("Turret/calcVelocity", velocity);
        
        return velocity;
    }

    public void setFlywheelVelocity(double mainVelocity, double upperVelocity) {
        turretIO.setFlywheelVelocity(mainVelocity, upperVelocity);
    }

    public void enableShooter(boolean enable) {
        turretEnabled = enable;
    }

    public boolean canShoot() {
        return turretInputs.areFlywheelsAtVelocity
            && turretInputs.isTurretAtTarget
            && !turretInputs.isTargetInKeepOut;
            // && shotStability < MAX_SHOT_STABILITY;
    }

    private Pose2d getTargetPoint(TurretAimTarget target) {
        if (DriverStation.getAlliance().isEmpty()){
            // no aliance color found yet, probably simulation
            return new Pose2d();
        }

        Alliance currentAlliance = DriverStation.getAlliance().get();
        Pose2d pose = new Pose2d();

        switch (target) {
            case Hub: {
                double x = 4.626;
                double y = 4.031;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
            case LobLower: {
                double x = 2;
                double y = 2;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
            case LobUpper: {
                double x = 2;
                double y = aprilTagLayout.getFieldWidth() - 2;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
        }
        return pose;
    }

    public void climbMode(boolean enable) {
        this.climbModeEnabled = enable;
    }
}