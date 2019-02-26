package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.slidedrive.HardwareSlide;
import org.firstinspires.ftc.teamcode.util.path.AStarPathFinder;
import org.firstinspires.ftc.teamcode.util.path.Path;
import org.firstinspires.ftc.teamcode.util.path.PathFinder;
import org.firstinspires.ftc.teamcode.util.path.test.GameMap;
import org.firstinspires.ftc.teamcode.util.path.test.PathTest;
import org.firstinspires.ftc.teamcode.util.path.test.UnitMover;

@Autonomous(name="Path Finder Test", group="Test")
public class PathFinderTest extends LinearOpMode {

    /* Private OpMode members */
    private ElapsedTime runtime = new ElapsedTime();

    /* Robot hardware */
    private HardwareSlide robot = new HardwareSlide(this);

    /** The map on which the units will move */
    private GameMap map = new GameMap();
    /** The path finder we'll use to search our map */
    private PathFinder finder;
    /** The last path found for the current unit */
    private Path path;

    /** The x coordinate of selected unit or -1 if none is selected */
    private int selectedx = -1;
    /** The y coordinate of selected unit or -1 if none is selected */
    private int selectedy = -1;

    /** The x coordinate of the target of the last path we searched for - used to cache and prevent constantly re-searching */
    private int lastFindX = -1;
    /** The y coordinate of the target of the last path we searched for - used to cache and prevent constantly re-searching */
    private int lastFindY = -1;

    private int step = 0;

    private void handleMousePressed(int x, int y) {
        x -= 50;
        y -= 50;
        x /= 16;
        y /= 16;

        if ((x < 0) || (y < 0) || (x >= map.getWidthInTiles()) || (y >= map.getHeightInTiles())) {
            return;
        }

        if (map.getUnit(x, y) != 0) {
            selectedx = x;
            selectedy = y;
            lastFindX = - 1;
        } else {
            if (selectedx != -1) {
                map.clearVisited();
                path = finder.findPath(new UnitMover(map.getUnit(selectedx, selectedy)),
                        selectedx, selectedy, x, y);

                if (path != null) {
                    path = null;
                    int unit = map.getUnit(selectedx, selectedy);
                    map.setUnit(selectedx, selectedy, 0);
                    map.setUnit(x,y,unit);
                    selectedx = x;
                    selectedy = y;
                    lastFindX = - 1;
                }
            }
        }
    }

    /**
     * Runs the autonomous routine.
     */
    @Override
    public void runOpMode() {
        // Initialize robot
//        robot.init(hardwareMap);
//        robot.drivetrain.encoderInit();
//        robot.imuInit(hardwareMap);

        finder = new AStarPathFinder(map, 500, true);
        handleMousePressed(20, 20);

        // Wait until we're told to go
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();  // Start counting run time from now.

        while (opModeIsActive()) {
            telemetry.addData("Status", path.getLength());
            telemetry.addData("Status", path);

        }
        if (isStopRequested() || !opModeIsActive()) { }
    }
}
