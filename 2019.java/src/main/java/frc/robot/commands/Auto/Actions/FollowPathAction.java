package frc.robot.commands.Auto.Actions;

import frc.robot.subsystems.Drivebase;
import frc.utility.PurePursuit.Path;

/**
 * Action for following a path defined by a Path object.
 * 
 * @see Drive
 * @see Path
 */
public class FollowPathAction implements Action {

    private Drivebase mDrive = Drivebase.getInstance();

    private Path mPath;
    private boolean mReversed;
    private boolean mHasStarted;

    public FollowPathAction(Path path, boolean reversed) {
        mPath = path;
        mReversed = reversed;
        mHasStarted = false;
    }

    @Override
    public boolean isFinished() {
        boolean done = mDrive.isFinishedPath() && mHasStarted;
        if (done) {
            System.out.println("Finished path");
        }
        return done;
    }

    @Override
    public void update() {
        mHasStarted = true;
    }

    @Override
    public void done() {
        mDrive.StopDrivetrain();
    }

    @Override
    public void start() {
        mDrive.followPath(mPath, mReversed);
    }

}
