package frc.robot.commands.IntakeCommands;

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
        intake.setIntakeAnglePID(130);
        intake.intakeSpeed(10, 10);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        //intake.intakeStop();
        //SmartDashboard.putBoolean("RunIntake", false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}