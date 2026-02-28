// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;


  public static final double robotWidth = 0.7112; // 28in
  public static final double robotLength = 0.6858; // 27in

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

}
