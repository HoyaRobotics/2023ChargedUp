package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePositionPID extends CommandBase {
  private final Intake intake;
  private final IntakingState intakingState;
  private final PIDController positionController = new PIDController(0.5, 0, 0);
  private final double setpoint =-1; //change this value

  public IntakePositionPID(Intake intake, IntakingState intakingState) {
    this.intake = intake;
    this.intakingState = intakingState;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    positionController.setSetpoint(setpoint);
  }

  @Override
  public void execute() {
    if(intake.getIntakeRetractorPosition() < positionController.getSetpoint()) {
      positionController.setP(8);
    }else{
      positionController.setP(0.6);
    }

    intake.intakeRotationSpeed(positionController.calculate(intake.getIntakeRetractorPosition()));
    if(intakingState == IntakingState.INTAKE){
      intake.intakeSpeed(0.60, 0.60);
    }else{
      intake.intakeSpeed(-0.65, -0.65);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
    intake.retractorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public enum IntakingState {
    INTAKE, OUTTAKE
  }
}