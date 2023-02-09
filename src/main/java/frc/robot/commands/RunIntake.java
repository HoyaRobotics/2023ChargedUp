package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    public final Intake intake;
    public final double frontVoltage;
    public final double backVoltage;

    public RunIntake(Intake intake, double frontVoltage, double backVoltage) {
        this.intake = intake;
        this.frontVoltage = frontVoltage;
        this.backVoltage = backVoltage;

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
