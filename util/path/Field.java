package org.firstinspires.ftc.teamcode.util.path;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.Steps;

import java.util.ArrayList;
import java.util.List;

public class Field {
    public int[][] fieldMap = {
            {2, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3},
            {2, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3},
            {2, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3},
            {2, 2, 2, 1, 0, 4, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0},
            {2, 2, 1, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0},
            {1, 1, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 1, 1},
            {0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 1, 2, 2},
            {0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 4, 0, 1, 2, 2, 2},
            {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 2},
            {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 2, 2},
            {3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 2, 2}
    };
    public int[][] startingPoint;
    public int[][] endPoint;

    public Field() {
    }

    public void init(Boolean alliance, Boolean startingPt, Boolean crater, Boolean depot) {
        if (crater == true && depot == false) {

        } else if (crater == false && depot == true) {

        } else if (crater == true && depot == true) {

        }
    }

    public void goToPoint(int[][] startingCoords, Steps.State steps) {

    }
}