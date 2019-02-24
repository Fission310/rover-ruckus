package org.firstinspires.ftc.teamcode.util.signals;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This manager handles the robot controller's background color and makes changing color easier and cleaner
 */
public class BackgroundColorManager {
    View relativeLayout;

    public void init(HardwareMap hwMap) {
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
    }

    public void setOrangeBackground() {
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, ColorConstants.orangeHSVColor));
            }
        });
    }

    public void setGreenBackground() {
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, ColorConstants.greenHSVColor));
            }
        });
    }

    public void resetBackgroundColor() {
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
