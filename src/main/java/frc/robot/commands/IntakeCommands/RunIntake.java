package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
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
        intake.setIntakeAnglePID(-31000);
        intake.intakeSpeed(0.6, 0.6);
        //GlobalVariables.fieldRelative = false;
        //0.8 top 0.8 bottom with top roller
        //0.7 top 0.7 bottom without top roller
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
