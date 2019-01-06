package org.firstinspires.ftc.teamcode.util.path;

public class Tile {
    public double[][] tile;

    public Tile() {
        init();
    }

    private void init() {
        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 24; j++) {
                tile[i][j] = 1.00;
            }
        }
    }

}

