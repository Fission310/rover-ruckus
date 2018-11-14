package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SoundManager {
    public int getSoundID(HardwareMap hwMap, String soundName) {
        return hwMap.appContext.getResources().getIdentifier(soundName,   "raw", hwMap.appContext.getPackageName());
    }

    public void playSound(Context context, int soundId) {
        SoundPlayer.getInstance().startPlaying(context, soundId);
    }
}
