package frc.robot.subsystems.climber;

public class ClimberConstants {
    public static final double EXTENSION_HOME_POSITION = 0;
    public static final double EXTENSION_METERS_PER_ROTATION = 0.0014;
    public static final double EXTENSION_EXTENDED_L1_POSITION = 15 * 4.8;
    public static final double EXTENSION_EXTENDED_L3_POSITION = 15 * 6.60;
    // threshold in motor rotations to consider extension done
    public static final double EXTENSION_PID_THRESHOLD = 2;

    public static final double CLIMBER_HOME_POSITION = 0;
    public static final double CLIMBER_L1_POSITION = 250;
    public static final double CLIMBER_L3_POSITION = 815;
    public static final double CLIMBER_ROATATION_DEG_PER_MOTOR_ROTATION = 0.33;

    // threshold in motor rotations to consider climb done
    public static final double CLIMBER_PID_THRESHOLD = 2;
    
    public static final double EXTENSION_HOME_SPEED = -0.2;
    public static final double CLIMBER_HOME_SPEED = -0.8;
}
