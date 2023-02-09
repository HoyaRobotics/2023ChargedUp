package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    public final Intake intake;
    public final double voltage;

    public RunIntake(Intake intake, double voltage) {
        this.intake = intake;
        this.voltage = voltage;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("RunIntake", true);
    }
    @Override
    public void execute() {
        intake.intakeStop();
        SmartDashboard.putBoolean("RunIntake", false);
    }
}
