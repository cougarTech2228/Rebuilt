package frc.robot.subsystems.climber;

public class ClimberConstants {
    public static final double EXTENSION_HOME_POSITION = 0;
    public static final double EXTENSION_EXTENDED_L1_POSITION = 46 * 4.8;
    public static final double EXTENSION_EXTENDED_L3_POSITION = 46 * 7;
    // threshold in motor rotations to consider extension done
    public static final double EXTENSION_PID_THRESHOLD = 2;

    public static final double CLIMBER_L1_POSITION = 400;
    public static final double CLIMBER_L3_POSITION = 820;

    // threshold in motor rotations to consider climb done
    public static final double CLIMBER_PID_THRESHOLD = 2;
    
    public static final double EXTENSION_HOME_SPEED = -0.2;
    public static final double CLIMBER_HOME_SPEED = -0.2;
}
