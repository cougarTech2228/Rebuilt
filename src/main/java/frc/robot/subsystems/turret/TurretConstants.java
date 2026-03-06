package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TurretConstants {
    public static final double flywheelVoltage = 3.0;

    public static Transform3d TurretOffset =
        new Transform3d(-0.18, 0.19, 0.72, new Rotation3d());

    public static final double MAX_FLYWHEEL_SPEED = 100;
    public static final double MIN_FLYWHEEL_SPEED = 10;

    public static final double HOOD_MIN_ANGLE = 0;
    public static final double HOOD_MAX_ANGLE = 1.4;

    // These are are safe rotation angles for the turret
    // beyond those would cause physical interference
    public static final double TURRET_MIN_ROTATION = -10.5;
    public static final double TURRET_MAX_ROTATION = 205.0;

    public static final double ENCODER_31T_OFFSET = 0.342773;
    public static final double ENCODER_37T_OFFSET = -0.053711;
    public static final double ENCODER_HOOD_MAGNET_OFFSET = -0.058105;

    public static final double HOOD_GEAR_RATIO = (85/12);
}
