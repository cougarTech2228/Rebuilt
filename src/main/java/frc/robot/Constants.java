package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double NEO_550_FREE_SPEED = 11000;
  public static final double FALCON_500_FREE_SPEED = 6380;
  public static final double KRAKEN_X44_FREE_SPEED = 7758;

  public static final double NEO_550_KV = 1 / NEO_550_FREE_SPEED;
  public static final double FALCON_500_KV = 1 / FALCON_500_FREE_SPEED;
  public static final double KRAKEN_X44_KV = 1 / KRAKEN_X44_FREE_SPEED;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // CAN ID constants
  public static final int CAN_ID_PIGEON = 10;

  public static final int CAN_ID_FL_DRIVE = 11;
  public static final int CAN_ID_FL_STEER = 12;
  public static final int CAN_ID_FL_ENCODER = 13;

  public static final int CAN_ID_FR_DRIVE = 14;
  public static final int CAN_ID_FR_STEER = 15;
  public static final int CAN_ID_FR_ENCODER = 16;

  public static final int CAN_ID_BL_DRIVE = 17;
  public static final int CAN_ID_BL_STEER = 18;
  public static final int CAN_ID_BL_ENCODER = 19;

  public static final int CAN_ID_BR_DRIVE = 20;
  public static final int CAN_ID_BR_STEER = 21;
  public static final int CAN_ID_BR_ENCODER = 22;

  public static final int CAN_ID_TURRET_MOTOR = 23;
  public static final int CAN_ID_TURRET_ENCODER_31T = 24;
  public static final int CAN_ID_TURRET_ENCODER_37T = 25;

  public static final int CAN_ID_CLIMBER_MAIN = 26;
  public static final int CAN_ID_CLIMBER_AUX = 37;
  public static final int CAN_ID_CLIMBER_EXTEND = 27;

  public static final int CAN_ID_TURRET_HOOD_ENCODER = 28;
  public static final int CAN_ID_TURRET_MOTOR_FLYWHEEL = 29;
  public static final int CAN_ID_UPPER_FLYWHEEL_MOTOR = 30;
  public static final int CAN_ID_TURRET_HOOD_MOTOR = 31;

  public static final int CAN_ID_INDEXER_MOTOR = 32;
  public static final int CAN_ID_KICKER_MOTOR = 33;

  public static final int CAN_ID_INTAKE_MOTOR = 34;
  public static final int CAN_ID_INTAKE_ANGLE_MOTOR = 35;
  public static final int CAN_ID_INTAKE_ENCODER = 36;

  // DIOs
  public static final int DIO_CLIMBER_READY = 0;
}
