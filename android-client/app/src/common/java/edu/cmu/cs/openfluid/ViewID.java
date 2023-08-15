package edu.cmu.cs.openfluid;

public enum ViewID {
    ARROW_UP(0),
    ARROW_DOWN(1),
    ARROW_LEFT(2),
    ARROW_RIGHT(3),
    FULL_SCREEN(4),
    CAM_CONTROL(5),
    AR_VIEW(6),
    ALIGN_CENTER(7),
    MENU(8),
    SCENE_LIST(9),
    RESET(10),
    PLAY_PAUSE(11),
    PARTICLE(12),
    AUTO_PLAY(13),
    ROTATE(14),
    INFO(15),
    HELP(16),
    MAIN(17),
    SIZE(18);

    private final int value;

    ViewID(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}

