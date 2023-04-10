package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeCone extends CommandBase {
    public final Intake intake;

    public RunIntakeCone(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeAnglePID(-32000);
        intake.intakeSpeed(0.5, 0.6);
        //GlobalVariables.fieldRelative = false;
        //0.8 top 0.8 bottom with top roller
        //0.7 top 0.7 bottom without top roller
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
