package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    public final Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("RunIntake", true);
    }

    @Override
    public void execute() {
        intake.intakeSpeed(1, 1);
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeStop();
        SmartDashboard.putBoolean("RunIntake", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
