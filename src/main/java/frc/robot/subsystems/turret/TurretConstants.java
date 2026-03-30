package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class TurretConstants {
    public static final double flywheelVoltage = 3.0;

    public static Transform2d TurretOffset =
        new Transform2d(-0.112, 0.125, new Rotation2d());
    public static final double MAX_FLYWHEEL_SPEED = 100;
    public static final double MIN_FLYWHEEL_SPEED = 10;

    public static final double HOOD_MIN_ANGLE = 0;
    public static final double HOOD_MAX_ANGLE = 1.23;

    public static final double turretTestIntercept = 34.298;

    // These are are safe rotation angles for the turret
    // beyond those would cause physical interference
    public static final double TURRET_MIN_ROTATION = -37;
    public static final double TURRET_MAX_ROTATION = 203;

    public static final double ENCODER_31T_OFFSET = 0.139404;
    public static final double ENCODER_37T_OFFSET = -0.163574;
    public static final double ENCODER_HOOD_MAGNET_OFFSET = 0.039795;

    public static final double HOOD_GEAR_RATIO = (85/12);
}
