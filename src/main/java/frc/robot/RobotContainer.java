// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.pathplanner.StartFiringCommand;
import frc.robot.commands.pathplanner.StopFiringCommand;

import frc.robot.commands.pathplanner.SpitCommand;
import frc.robot.commands.AutoClimbL1Command;
import frc.robot.commands.AutoClimbL3Command;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DescendCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ExtendClimberCommand;
import frc.robot.commands.HomeClimberCommand;
import frc.robot.commands.IntakeSpitCommand;
import frc.robot.commands.OscillateIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOMotor;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOMotors;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.Intake.OscillateType;
import frc.robot.subsystems.hopper.HopperIOMotors;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOMotors;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  final Drive drive;
  final Vision vision;
  final Hopper hopper;
  final Turret turret;
  final Climber climber;
  final Intake intake;
 
  public static final CANBus kCanivore = new CANBus("canivore");
  public static final CANBus kRio = new CANBus("rio");

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private double driverOverridePercentage = 1.0;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  @AutoLogOutput(key = "ComponentPoses/Turret")
  public static Pose3d turretPose = new Pose3d(-0.11, 0.1241, 0.5, new Rotation3d());

  @AutoLogOutput(key = "ComponentPoses/Intake")
  public static Pose3d intakePose = new Pose3d(0.295, -0.3, 0.22, new Rotation3d());

  public static final double climberXBase = -0.277;
  @AutoLogOutput(key = "ComponentPoses/Climber")
  public static Pose3d climberPose = new Pose3d( climberXBase, -0.155, 0.68, new Rotation3d());

  private static final String EXTEND_CLIMBER_L1_KEY = "ExtendClimberL1";
  private static final String EXTEND_CLIMBER_L3_KEY = "ExtendClimberL3";
  private static final String CLIMB_L1_KEY = "ClimbL1";
  private static final String CLIMB_L3_KEY = "ClimbL3";
  private static final String UNCLIMB_KEY = "Unclimb";
  private static final String HOME_CLIMBER_KEY = "HomeClimber";
  private static final String CANCEL_AUTO_CLIMB_KEY = "Cancel Auto Climb";

  private final ExtendClimberCommand extendClimberL1Command;
  private final ExtendClimberCommand extendClimberL3Command;
  private final ClimbCommand climbL1Command;
  private final ClimbCommand climbL3Command;
  private final DescendCommand descendCommand;
  private final HomeClimberCommand homeClimberCommand;

  private final AutoClimbL3Command autoClimbL3Command;

  private final ToggleIntakeCommand toggleIntakeCommand;
  private final IntakeSpitCommand intakeSpitCommand;
  private final Command startFiringCommand;
  private final Command stopFiringCommand;
  private final Command spitCommand;
  private final ShootCommand shootCommand;
  private final OscillateIntakeCommand oscillateIntakeCommand;
  private final CancelCommand cancelAutoClimbCommand;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        vision = new Vision(
            drive::addVisionMeasurement, drive::getRotationAtTime,
            new VisionIOPhotonVision(frontCameraName, robotToFrontCamera),
            new VisionIOPhotonVision(leftCameraName, robotToLeftCamera),
            new VisionIOPhotonVision(backCameraName, robotToBackCamera),
            new VisionIOPhotonVision(rightCameraName, robotToRightCamera));

        turret = new Turret(new TurretIOMotors(), drive);
        hopper = new Hopper(new HopperIOMotors());
        climber = new Climber(new ClimberIOMotor());
        intake = new Intake(new IntakeIOMotors());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(
            drive::addVisionMeasurement, drive::getRotationAtTime,
            new VisionIOPhotonVisionSim(frontCameraName, robotToFrontCamera, drive::getPose),
            new VisionIOPhotonVisionSim(leftCameraName, robotToLeftCamera, drive::getPose),
            new VisionIOPhotonVisionSim(backCameraName, robotToBackCamera, drive::getPose),
            new VisionIOPhotonVisionSim(rightCameraName, robotToRightCamera, drive::getPose));

        turret = new Turret(new TurretIOSim(), drive);
        climber = new Climber(new ClimberIOSim());
        hopper = new Hopper(new HopperIOSim());
        intake = new Intake(new IntakeIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        vision = new Vision(drive::addVisionMeasurement, drive::getRotationAtTime, new VisionIO() {
        }, new VisionIO() {
        });
        turret = new Turret(new TurretIO() {
        }, drive);
        hopper = new Hopper(new HopperIO() {
        });
        climber = new Climber(new ClimberIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }

    startFiringCommand = new StartFiringCommand(hopper, turret, intake);
    stopFiringCommand = new StopFiringCommand(hopper, turret, intake);
    spitCommand = new SpitCommand(hopper, intake);
    toggleIntakeCommand = new ToggleIntakeCommand(intake, climber);

    autoClimbL3Command = new AutoClimbL3Command(drive, climber, turret, intake);

    oscillateIntakeCommand = new OscillateIntakeCommand(climber, intake);
    
    // Register Auto commands
    NamedCommands.registerCommand("StartFiringCommand", startFiringCommand);
    NamedCommands.registerCommand("StopFiringCommand", stopFiringCommand);
    NamedCommands.registerCommand("SpitCommand", spitCommand);
    NamedCommands.registerCommand("ToggleIntakeCommand", new ToggleIntakeCommand(intake, climber));
    NamedCommands.registerCommand("AutoClimbL1Command", new AutoClimbL1Command(drive, climber, turret, intake));
    NamedCommands.registerCommand("OscillateIntakeCommand", oscillateIntakeCommand);

    // Set up auto routines
    // Instead if automatically building options based on detected .auto files, create them manually
    // so that only autos we want to be able to choose are shown.
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());

    autoChooser.addOption("New Left Bump Double No Climb", AutoBuilder.buildAuto("New Comp Bump 4 Double"));

    autoChooser.addOption("Left Bump Climb", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Comp Bump 4 Climb"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Right Bump No Climb", AutoBuilder.buildAuto("Comp Bump 1 No Climb"));

    autoChooser.addOption("Right Bump Climb", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Comp Bump 1 Climb"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Center Climb Left", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Comp Center Climb Left"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Center Climb Right", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Comp Center Climb Right"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Left Bump Middle + Depot", AutoBuilder.buildAuto("Comp Bump 4 Outpost"));
    
    autoChooser.addOption("Hub Depot Climb", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Hub Depot Climb"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Right Bump Climb", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Comp Bump 1 Climb"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Right Straight Climb", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Bump 1 Straight Climb"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    autoChooser.addOption("Right Bump Double", new SequentialCommandGroup(
        AutoBuilder.buildAuto("Comp Bump 1 Double"),
        new AutoClimbL1Command(drive, climber, turret, intake)));

    extendClimberL1Command = new ExtendClimberCommand(climber, intake, ClimberLevel.L1, turret);
    extendClimberL3Command = new ExtendClimberCommand(climber, intake, ClimberLevel.L3, turret);
    climbL1Command = new ClimbCommand(climber, ClimberLevel.L1);
    climbL3Command = new ClimbCommand(climber, ClimberLevel.L3);
    descendCommand = new DescendCommand(climber);
    homeClimberCommand = new HomeClimberCommand(climber, turret);

    intakeSpitCommand = new IntakeSpitCommand(intake, climber, hopper);

    shootCommand = new ShootCommand(hopper, turret);

    cancelAutoClimbCommand = new CancelCommand(autoClimbL3Command);

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putBoolean("TestMode", false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * driverOverridePercentage,
            () -> -controller.getLeftX() * driverOverridePercentage,
            () -> -controller.getRightX() * driverOverridePercentage));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    
    controller.a().onTrue(autoClimbL3Command);

    controller.y().whileTrue(intakeSpitCommand);

    controller.rightBumper().onTrue(toggleIntakeCommand);

    controller.rightTrigger(0.5).whileTrue(shootCommand);
    controller.leftTrigger(0.5).whileTrue(oscillateIntakeCommand.repeatedly());
    
    controller.leftBumper()
        .onTrue(new InstantCommand(() -> {
            driverOverridePercentage = 0.5;
        }))
        .onFalse(new InstantCommand(() -> {
            driverOverridePercentage = 1.0;
        }));
    controller.povUp().onTrue(new InstantCommand(() -> {
        intake.oscillate(OscillateType.ONCE);
    }));
    controller.povDown().onTrue(new InstantCommand(() -> {
        intake.oscillate(OscillateType.WAVE);
    }));

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putBoolean(HOME_CLIMBER_KEY, false);
    SmartDashboard.putBoolean(EXTEND_CLIMBER_L1_KEY, false);
    SmartDashboard.putBoolean(EXTEND_CLIMBER_L3_KEY, false);
    SmartDashboard.putBoolean(CLIMB_L1_KEY, false);
    SmartDashboard.putBoolean(CLIMB_L3_KEY, false);
    SmartDashboard.putBoolean(UNCLIMB_KEY, false);
    SmartDashboard.putBoolean(CANCEL_AUTO_CLIMB_KEY, false);

    new Trigger(() -> SmartDashboard.getBoolean(EXTEND_CLIMBER_L1_KEY, false))
        .whileTrue( extendClimberL1Command
                    .andThen(new InstantCommand(() -> {
                        SmartDashboard.putBoolean(EXTEND_CLIMBER_L1_KEY, false);
                    }))
        );

    new Trigger(() -> SmartDashboard.getBoolean(EXTEND_CLIMBER_L3_KEY, false))
                .whileTrue( extendClimberL3Command
                    .andThen(new InstantCommand(() -> {
                        SmartDashboard.putBoolean(EXTEND_CLIMBER_L3_KEY, false);
                    }))
        );

    new Trigger(() -> SmartDashboard.getBoolean(CLIMB_L1_KEY, false))
                .whileTrue( climbL1Command
                    .andThen(new InstantCommand(() -> {
                        SmartDashboard.putBoolean(CLIMB_L1_KEY, false);
                    }))
        );

    new Trigger(() -> SmartDashboard.getBoolean(CLIMB_L3_KEY, false))
                .whileTrue( climbL3Command
                    .andThen(new InstantCommand(() -> {
                        SmartDashboard.putBoolean(CLIMB_L3_KEY, false);
                    }))
        );

    new Trigger(() -> SmartDashboard.getBoolean(UNCLIMB_KEY, false))
                .whileTrue( descendCommand
                    .andThen(new InstantCommand(() -> {
                        SmartDashboard.putBoolean(UNCLIMB_KEY, false);
            }))
        );

    new Trigger(() -> SmartDashboard.getBoolean(HOME_CLIMBER_KEY, false))
        .whileTrue( homeClimberCommand
            .andThen(new InstantCommand(() -> {
                SmartDashboard.putBoolean(HOME_CLIMBER_KEY, false);
            }))
        );

    new Trigger(() -> SmartDashboard.getBoolean(CANCEL_AUTO_CLIMB_KEY, false))
        .whileTrue( cancelAutoClimbCommand
            .andThen(new InstantCommand(() -> {
                SmartDashboard.putBoolean(CANCEL_AUTO_CLIMB_KEY, false);
            }))
        );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simulationPeriodic() {
  }

  public void testPeriodic() {
  }

  public void teleopInit() {
    turret.climbMode(false);
  }

  public void teleopPeriodic() {
    if (SmartDashboard.getBoolean("TestMode", false)) {
          intake.setIntakeAngle(SmartDashboard.getNumber("IntakePosition", 1.0));
    }
  }

  public void autonomousPeriodic() {
  }

}