package frc.robot.subsystems;

import frc.robot.GlobalVariables;

public class SelectPosition {
    public void SelectPosition() {

    }

    public void rightOfPosition() {
        GlobalVariables.leftRightPosition = GlobalVariables.leftRightPosition + 1;
        if (GlobalVariables.leftRightPosition == 10) {
            GlobalVariables.leftRightPosition = 1;
        }
    }

    public void leftOfPosition() {
        GlobalVariables.leftRightPosition = GlobalVariables.leftRightPosition - 1;
        if (GlobalVariables.leftRightPosition == 0) {
            GlobalVariables.leftRightPosition = 9;
        }
    }

    public void upFromPosition() {
        GlobalVariables.upDownPosition = GlobalVariables.upDownPosition + 1;
        if (GlobalVariables.upDownPosition == 4) {
            GlobalVariables.upDownPosition = 1;
        }
    }

    public void downFromPosition() {
        GlobalVariables.upDownPosition = GlobalVariables.upDownPosition + 1;
        if (GlobalVariables.upDownPosition == 0) {
            GlobalVariables.upDownPosition = 3;
        }
    }
}
