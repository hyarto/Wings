package com.hyarto.wings;

import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.view.WindowManager;

import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.view.WindowInsetsControllerCompat;

import com.google.androidgamesdk.GameActivity;

/** STRATO-32 的 Android 入口：GameActivity + 沉浸式全屏 + 常亮；游戏逻辑全部在 Rust（libapp_client.so）。 */
public class MainActivity extends GameActivity {
    static {
        System.loadLibrary("c++_shared");
        System.loadLibrary("app_client");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        // 让内容延伸到刘海区域并保持屏幕常亮（飞行模拟没有触屏以外的输入，系统会误熄屏）
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        super.onCreate(savedInstanceState);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            hideSystemUi();
        }
    }

    private void hideSystemUi() {
        View decor = getWindow().getDecorView();
        WindowInsetsControllerCompat c = WindowCompat.getInsetsController(getWindow(), decor);
        if (c != null) {
            c.setSystemBarsBehavior(WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
            c.hide(WindowInsetsCompat.Type.systemBars());
        }
    }
}
