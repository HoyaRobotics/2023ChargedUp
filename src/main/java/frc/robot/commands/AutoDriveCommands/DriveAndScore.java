package frc.robot.commands.AutoDriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.commands.ArmCommands.PlaceOnPosition;
import frc.robot.commands.ArmCommands.ReleaseAndRetract;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pincher;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAndScore extends SequentialCommandGroup {
    public DriveAndScore(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator, Arm arm, Pincher pincher) {
        AddCommands(
            new DriveToSelectedPeg(swerveSubsystem, poseEstimator),
            swerveSubsystem.createCommandForTrajectory(GlobalVariables.trajectory),
            new PlaceOnPosition(arm, pincher, () -> GlobalVariables.upDownPosition),
            new ReleaseAndRetract(pincher, arm, () -> GlobalVariables.upDownPosition)
        );
    }

    private void AddCommands(DriveToSelectedPeg driveToSelectedPeg, Command createCommandForTrajectory,
            PlaceOnPosition placeOnPosition, ReleaseAndRetract releaseAndRetract) {
    }
}
