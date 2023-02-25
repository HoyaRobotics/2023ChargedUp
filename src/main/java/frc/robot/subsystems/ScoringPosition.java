package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;

public class ScoringPosition extends SubsystemBase {
    private int[] currentPosition;
    
    public ScoringPosition() {
        this.currentPosition = GlobalVariables.currentPosition;
    }

    public void rightOfPosition() {
        currentPosition[1] = currentPosition[1] + 1;
        if(currentPosition[1] == 10) {
            currentPosition[1] = 1;
        }
        GlobalVariables.currentPosition = currentPosition;
    }

    public void leftOfPosition() {
        currentPosition[1] = currentPosition[1] - 1;
        if(currentPosition[1] == 0) {
            currentPosition[1] = 9;
        }
        GlobalVariables.currentPosition = currentPosition;
    }

    public void upOnePosition() {
        currentPosition[0] = currentPosition[0] + 1;
        if(currentPosition[0] == 4) {
            currentPosition[0] = 1;
        }
        GlobalVariables.currentPosition = currentPosition;
    }

    public void downOnePosition() {
        currentPosition[0] = currentPosition[0] - 1;
        if(currentPosition[0] == 0) {
            currentPosition[0] = 3;
        }
        GlobalVariables.currentPosition = currentPosition;
    }
}
