package org.firstinspires.ftc.teamcode.xendy;

import java.io.Serializable;
import java.util.ArrayList;

public class PathData implements Serializable {
    public String name;
    public ArrayList<SaveState> states;
    public PathData(String name, ArrayList<SaveState> states) {
        this.states = states;
        this.name = name;
    }
}
