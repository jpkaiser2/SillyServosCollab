package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AutoPathStateMachine {

    public enum State {
        IDLE,
        RUNNING,
        COMPLETE
    }

    private final Follower follower;
    private final List<PathChain> paths = new ArrayList<>();
    private int index = -1;
    private State state = State.IDLE;

    public AutoPathStateMachine(Follower follower) {
        this.follower = follower;
    }

    public AutoPathStateMachine add(PathChain path) {
        if (path != null) paths.add(path);
        return this;
    }

    public AutoPathStateMachine addAll(List<PathChain> pathList) {
        if (pathList != null) paths.addAll(pathList);
        return this;
    }

    public void start() {
        if (paths.isEmpty()) {
            state = State.COMPLETE;
            return;
        }
        index = 0;
        follower.followPath(paths.get(index));
        state = State.RUNNING;
    }

    public void update() {
        if (state != State.RUNNING) return;

        if (!follower.isBusy()) {
            index++;
            if (index < paths.size()) {
                follower.followPath(paths.get(index));
            } else {
                state = State.COMPLETE;
            }
        }
    }

    public void reset() {
        index = -1;
        state = State.IDLE;
    }

    public State getState() {
        return state;
    }

    public int getIndex() {
        return index;
    }

    public int size() {
        return paths.size();
    }

    public boolean isComplete() {
        return state == State.COMPLETE;
    }

    public List<PathChain> getPaths() {
        return Collections.unmodifiableList(paths);
    }
}
