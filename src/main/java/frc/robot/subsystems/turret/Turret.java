package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private double currentHoodAngle = 0;

    public Turret(TurretIO turretIO, Drive driveSubsystem) {
        this.turretIO = turretIO;
        this.driveSubsystem = driveSubsystem;

        SmartDashboard.putNumber("TurretHoodElevation", 0.0);
        SmartDashboard.putNumber("TurretFlywheelVelocity", 0.0);
        SmartDashboard.putNumber("TurretTestDistance", 0.0);
        SmartDashboard.putNumber("TurretTestFlywheelRatio", 1.0);

        SmartDashboard.putNumber("TurretAngle", 0.0);
        SmartDashboard.putNumber("TurretTestX", 4.6129);
        SmartDashboard.putNumber("TurretTestIntercept", 28);
        SmartDashboard.putNumber("TurretTestAngleIntercept", 0.7);

        SmartDashboard.setPersistent("TurretTestX");
        SmartDashboard.setPersistent("TurretTestIntercept");
        SmartDashboard.setPersistent("TurretTestAngleIntercept");
        // Shoot-on-the-move tuning parameter. Represents the average m/s of the fuel
        // across its entire flight path.
        SmartDashboard.putNumber("TurretShotSpeedMpS", 1.9);
    }

    private double getTargetDistance() {
        Pose2d robotPose = driveSubsystem.getPose();
        // Return distance to the virtual compensated target, not the physical hub
        return robotPose.getTranslation().getDistance(virtualTargetPose.getTranslation());
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);

        // --- Shoot On The Move Math ---
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d realTargetPose = getTargetPoint(aimTarget);

        // Get the robot's velocity and make it field-relative
        ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
        Translation2d fieldRelativeVelocity = new Translation2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond
        ).rotateBy(robotPose.getRotation());

        // Estimate time of flight
        double realDistance = robotPose.getTranslation().getDistance(realTargetPose.getTranslation());
        double shotVelocity = SmartDashboard.getNumber("TurretShotSpeedMpS", 15.0);
        double timeOfFlight = realDistance / shotVelocity;

        // Offset the target point by the velocity we will impart during the time of flight
        Translation2d movingOffset = fieldRelativeVelocity.times(timeOfFlight);
        virtualTargetPose = new Pose2d(
            realTargetPose.getTranslation().minus(movingOffset),
            realTargetPose.getRotation()
        );

        turretInputs.targetDistance = getTargetDistance();

        // Log the real target vs the virtual aim target for easy debugging in AdvantageScope
        Logger.recordOutput("Turret/RealTargetPose", realTargetPose);
        Logger.recordOutput("Turret/VirtualTargetPose", virtualTargetPose);
        Logger.processInputs("Turret", turretInputs);

        RobotContainer.turretPose = new Pose3d(
            RobotContainer.turretPose.getX(),
            RobotContainer.turretPose.getY(),
            RobotContainer.turretPose.getZ(),
            new Rotation3d(turretInputs.turretAngle));
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
        // Point the turret at the compensated virtual target
        Pose2d targetPose = virtualTargetPose;

        turretInputs.turretTargetPoint = targetPose;

        // Get the Translation (X, Y) of both poses
        Translation2d robotXY = robotPose.getTranslation().plus(TurretConstants.TurretOffset.getTranslation().toTranslation2d());
        Translation2d targetXY = targetPose.getTranslation();

        // Calculate the difference vector (Vector from Robot -> Target)
        Translation2d difference = targetXY.minus(robotXY);

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

    private double getAngleForTarget() {
        double distance = 0;

        double turretTestDistance = SmartDashboard.getNumber("TurretTestDistance", 0.0);
        if (turretTestDistance > 0) {
            distance = turretTestDistance;
        } else {
            distance = getTargetDistance();
        }

        double angle;

        if (aimTarget == TurretAimTarget.Hub) {
            // y = 0.0099x3 - 0.1406x2 + 0.6786x - 0.6695
            double turretTestAngleX = SmartDashboard.getNumber("TurretTestAngleIntercept", 0.7);

            angle = (0.0099 * distance * distance * distance) -
                    (0.1406 * distance * distance) +
                    (0.6786 * distance) - turretTestAngleX;

        } else {
            angle = 1.3;
        }

        // sanity check the values
        angle = Math.min(angle, TurretConstants.HOOD_MAX_ANGLE);
        angle = Math.max(angle, TurretConstants.HOOD_MIN_ANGLE);

         Logger.recordOutput("Turret/RealAngle", angle);
        return angle;
    }

    // private double getRatioForTarget() {
    //     // y = 5E-09x3 - 7E-06x2 + 0.0029x + 0.6728
    // }

    private double getVelocityForTarget(boolean test) {
        double distance = 0;

        if (test) {
            double turretTestDistance = SmartDashboard.getNumber("TurretTestDistance", 0.0);
            distance = turretTestDistance;
         } else {
            distance = getTargetDistance();
        }

        double velocity;

        if (aimTarget == TurretAimTarget.Hub) {
            double turretTestX = SmartDashboard.getNumber("TurretTestX", 4.85);
            double TurretTestIntercept = SmartDashboard.getNumber("TurretTestIntercept", 28.5);
            //y = 4.6129x + 29.763
            velocity = (turretTestX * distance) + TurretTestIntercept;

        } else {
            //y = 5.2631x + 18.264
            velocity = (5.2631 * distance) + 18.264;
        }

        // cap the speed to an achievable value
        velocity = Math.min(velocity, TurretConstants.MAX_FLYWHEEL_SPEED);

        // ensure we don't end up with a negative value or super slow speed
        velocity = Math.max(velocity, TurretConstants.MIN_FLYWHEEL_SPEED);

        return velocity;
    }

    public void setFlywheelVelocity(double mainVelocity, double upperVelocity) {
        turretIO.setFlywheelVelocity(mainVelocity, upperVelocity);
    }

    public void enableShooter(boolean enable) {
        if (SmartDashboard.getBoolean("TestMode", false)) {
            if (enable) {
                setHoodElevation(SmartDashboard.getNumber("TurretHoodElevation", 0.0));
                double ratio = SmartDashboard.getNumber("TurretTestFlywheelRatio", 1.0);
                double velocity = SmartDashboard.getNumber("TurretFlywheelVelocity", 0.0);
                setFlywheelVelocity(velocity, ratio * velocity);

                setAimTarget(SmartDashboard.getNumber("TurretAngle", 0.0));
            } else {
                setFlywheelVelocity(0, 0);
            }
            // boolean indexerTest = SmartDashboard.getBoolean("IndexerTest", false);
            // if (indexerTest) {
            //   hopper.indexerOn(true);
            //   hopper.kickerOn(true);
            // } else {
            //   hopper.indexerOff();
            //   hopper.kickerOff();
            // }
            // intake.setIntakeAngle(SmartDashboard.getNumber("IntakePosition", 1.0));
            // intake.setIntakeVelocity(SmartDashboard.getNumber("IntakeVelocity", 1.0));
        } else {
            if (enable) {
                double vel = getVelocityForTarget(false);
                setFlywheelVelocity(vel, vel * 1.2);
                double angle = getAngleForTarget();
                if (Math.abs(angle - currentHoodAngle) > 0.01) {
                    currentHoodAngle = angle;
                    setHoodElevation(getAngleForTarget());
                }
            } else {
                setFlywheelVelocity(0, 0);
                // no need to move the hood on disable
            }
        }
    }

    public boolean canShoot() {
        return turretInputs.areFlywheelsAtVelocity && turretInputs.isTurretAtTarget && !turretInputs.isTargetInKeepOut;
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
                double y = 1;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
            case LobUpper: {
                double x = 2;
                double y = aprilTagLayout.getFieldWidth() - 1;
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