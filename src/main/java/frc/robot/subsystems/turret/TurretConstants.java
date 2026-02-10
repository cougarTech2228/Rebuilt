package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TurretConstants {
    public static final double flywheelVoltage = 3.0;

    public static Transform3d TurretOffset =
        new Transform3d(-0.18, 0.19, 0.72, new Rotation3d());

    public static final double MAX_FLYWHEEL_SPEED = 90;

    // [New Constant] Tuning parameter. Start at 1.0. 
    // If you miss "behind" the target while moving, increase to 1.1 or 1.2 to account for air drag.
    public static final double MOTION_COMPENSATION_FACTOR = 1.0; 


}
