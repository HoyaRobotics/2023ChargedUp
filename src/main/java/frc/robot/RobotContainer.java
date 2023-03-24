// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Set up on Works laptop
//Eli Test
package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.PIDBalanceOnChargeStation;
import frc.robot.commands.IntakeCommands.ReverseIntake;
import frc.robot.commands.IntakeCommands.RunConveyor;
import frc.robot.commands.IntakeCommands.RunIntakeCone;
import frc.robot.commands.IntakeCommands.RunIntakeCube;
import frc.robot.commands.IntakeCommands.SideStationIntake;
import frc.robot.commands.IntakeCommands.StopConveyor;
import frc.robot.commands.IntakeCommands.ReverseConveyor;
import frc.robot.commands.SetPose;
import frc.robot.commands.ToggleFieldRelative;
import frc.robot.commands.ArmCommands.DropConeAgain;
import frc.robot.commands.ArmCommands.PlaceOnPosition;
import frc.robot.commands.ArmCommands.ReleaseAndRetract;
import frc.robot.commands.ArmCommands.AutoSpecific.GripConeAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.GripCubeAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.PlaceOnPositionAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.ReleaseOnPositionAuto;
import frc.robot.commands.ArmCommands.AutoSpecific.RetractFromPositionAuto;
import frc.robot.commands.ArmCommands.ComplexMethod.MoveArmExtensionToCubePickup;
import frc.robot.commands.AutoDriveCommands.DriveToClosestPeg;
import frc.robot.commands.AutoDriveCommands.DriveToLoadingStation;
import frc.robot.commands.IntakeCommands.StopIntake;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Classifier;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.Intake;

//Initial GitHub test
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, pigeon2Subsystem);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Arm arm = new Arm();
  private final Pincher pincher = new Pincher();
  private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();
  private final Classifier classifier = new Classifier();

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Use commands in auto file

    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
      swerveSubsystem,
      poseEstimator,
      () -> -driverController.getLeftX(),
      () -> -driverController.getLeftY(),
      () -> -driverController.getRightX(),
      () -> GlobalVariables.fieldRelative,
      () -> GlobalVariables.maxSpeed));

    // Configure the trigger bindings
    configureBindings();

    m_chooser.setDefaultOption("Nothing", "Nothing");
    m_chooser.addOption("Score", "Score");
    m_chooser.addOption("Level Right", "LevelRight");
    m_chooser.addOption("Score Level Left", "ScoreLevelLeft");
    m_chooser.addOption("Score Level Middle", "ScoreLevelMiddle");
    m_chooser.addOption("Score Level Right", "ScoreLevelRight");
    m_chooser.addOption("Score Cross Level Middle", "ScoreCrossLevelMiddle");
    m_chooser.addOption("Score Cross Pickup Level Left Middle", "ScoreCrossPickupLevelLeftMiddle");
    m_chooser.addOption("Score Cross Pickup Level Right Middle", "ScoreCrossPickupLevelRightMiddle");
    m_chooser.addOption("Score Pickup Level Left", "ScorePickupLevelLeft");
    m_chooser.addOption("Score Pickup Level Right", "ScorePickupLevelRight");
    m_chooser.addOption("Two Left", "TwoLeft");
    m_chooser.addOption("Two Right", "TwoRight");
    //m_chooser.addOption("Two Left Level", "TwoLeftFast");
    //m_chooser.addOption("Two Right Level", "TwoRightFast");
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.back().onTrue(new SetPose(poseEstimator, new Pose2d(0, 0, new Rotation2d(0))));
    driverController.x().onTrue(new ToggleFieldRelative());
    driverController.a().whileTrue(new PIDBalanceOnChargeStation(pigeon2Subsystem, swerveSubsystem, poseEstimator));
    driverController.rightBumper().onTrue(new RunIntakeCone(intake).alongWith(new RunConveyor(conveyor)));
    driverController.rightTrigger(0.5).onTrue(new ReverseIntake(intake).alongWith(new ReverseConveyor(conveyor)));
    driverController.rightBumper().onFalse(new StopIntake(intake).andThen(new WaitCommand(1).andThen(new StopConveyor(conveyor))));
    driverController.rightTrigger(0.5).onFalse(new StopIntake(intake).alongWith(new StopConveyor(conveyor)));
    driverController.leftBumper().onTrue(new RunIntakeCube(intake).alongWith(new RunConveyor(conveyor)));
    driverController.leftBumper().onFalse(new StopIntake(intake).andThen(new WaitCommand(1).andThen(new StopConveyor(conveyor))));
    driverController.y().onTrue(new SideStationIntake(intake).alongWith(new RunConveyor(conveyor)));
    driverController.y().onFalse(new StopIntake(intake).andThen(new StopConveyor(conveyor)));
    
    driverController.rightStick().onTrue(new DriveToClosestPeg(swerveSubsystem, poseEstimator, candleSubsystem, () -> -driverController.getLeftX(),
    () -> -driverController.getLeftY(),
    () -> -driverController.getRightX(),
    () -> GlobalVariables.fieldRelative,
    () -> GlobalVariables.maxSpeed))
    .onFalse(new InstantCommand(() -> {
      if(swerveSubsystem.getCurrentCommand() != null) {
        swerveSubsystem.getCurrentCommand().cancel();
        candleSubsystem.setGamePiece();
      }
    }));

    driverController.leftStick().onTrue(new DriveToLoadingStation(swerveSubsystem, poseEstimator, candleSubsystem, () -> -driverController.getLeftX(),
    () -> -driverController.getLeftY(),
    () -> -driverController.getRightX(),
    () -> GlobalVariables.fieldRelative,
    () -> GlobalVariables.maxSpeed))
    .onFalse(new InstantCommand(() -> {
      if(swerveSubsystem.getCurrentCommand() != null) {
        swerveSubsystem.getCurrentCommand().cancel();
        candleSubsystem.setGamePiece();
      }
    }));

    //operatorController.x().onTrue(new GripAndHold(pincher, arm));
    //operatorController.x().onTrue(new Toggle(pincher));
    operatorController.x().onTrue(new MoveArmExtensionToCubePickup(arm, pincher, classifier));
    operatorController.a().onTrue(new PlaceOnPosition(arm, pincher, () -> (GlobalVariables.upDownPosition)));
    operatorController.b().onTrue(new ReleaseAndRetract(pincher, arm, () -> (GlobalVariables.upDownPosition)));
    operatorController.y().onTrue(new DropConeAgain(arm, pincher));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> GlobalVariables.isCone = !GlobalVariables.isCone).alongWith(new InstantCommand(() -> candleSubsystem.setGamePiece())));
    operatorController.rightTrigger(0.5).onTrue(new RunConveyor(conveyor)).onFalse(new StopConveyor(conveyor));
    operatorController.leftTrigger(0.5).onTrue(new ReverseConveyor(conveyor)).onFalse(new StopConveyor(conveyor));
    
    operatorController.povUp().onTrue(new InstantCommand(() -> arm.moveGridTargetIf(true)));
    operatorController.povDown().onTrue(new InstantCommand(() -> arm.moveGridTargetIf(false)));
    operatorController.povRight().onTrue(new InstantCommand(() -> GlobalVariables.leftRightPosition++));
    operatorController.povLeft().onTrue(new InstantCommand(() -> GlobalVariables.leftRightPosition--));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /*
  Commented out for the time being uncomment when we are ready to add the command
  */

  public Command getAutonomousCommand() {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("RunIntakeCone", new RunIntakeCone(intake));
    eventMap.put("RunIntakeCube", new RunIntakeCube(intake));
    eventMap.put("RunConveyor", new RunConveyor(conveyor));
    eventMap.put("StopIntake", new StopIntake(intake));
    eventMap.put("StopConveyor", new StopConveyor(conveyor));
    eventMap.put("Level", new PIDBalanceOnChargeStation(pigeon2Subsystem, swerveSubsystem, poseEstimator));
    eventMap.put("Stop", new InstantCommand(() -> swerveSubsystem.stop(), swerveSubsystem));

    eventMap.put("GripCone", new GripConeAuto(pincher, arm));
    eventMap.put("GripCube", new GripCubeAuto(pincher, arm));
    eventMap.put("Place", new PlaceOnPositionAuto(arm, pincher, () -> 2));
    eventMap.put("Release", new ReleaseOnPositionAuto(arm, pincher, () -> 2));
    eventMap.put("Retract", new RetractFromPositionAuto(arm));


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      poseEstimator::getPose, // Pose2d supplier
      poseEstimator::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.SwerveConstants.KINEMATICS, // SwerveDriveKinematics
      new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true,
      swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    // An example command will be run in autonomous
    List<PathPlannerTrajectory> trajectories;
    if(m_chooser.getSelected() == "Nothing") {
      return null;
    }else{
      trajectories = PathPlanner.loadPathGroup(m_chooser.getSelected(), 3, 2);//vel 3, accel 2.5
      return autoBuilder.fullAuto(trajectories);
    }
    }
  }