package org.firstinspires.ftc.teamcode.subsystems;

public class LaunchSubsystem
{
    private final IndexerSubsystem indexer;
    private boolean launching;
    private String[] wantedPattern;
    private String[] indexerPattern;
    private int step;
    private boolean moving;
    private boolean pulsing;
    private int position;

    public LaunchSubsystem(IndexerSubsystem indexer)
    {
        this.indexer = indexer;
        launching = false;
        wantedPattern = new String[]{"green", "purple", "purple"};
        indexerPattern = new String[]{"empty", "empty", "empty"};
        step = 0;
        moving = false;
        pulsing = false;
        position = 0;
    }

    public void update()
    {
        // update based on step, even steps move to position, odd steps pulse lever arm to launch
        if (launching)
        {
            if (step == 0)
            {
                if (!moving)
                    goToNextPosition(1);
                else if (moving && !indexer.isMoving())
                {
                    moving = false;
                    step++;
                }
            }
            else if (step == 1)
            {
                if (!pulsing) {
                    indexer.handleLeverButton(true);
                    pulsing = true;
                } else if (pulsing && !indexer.getLeverState())
                {
                    pulsing = false;
                    step++;
                    indexerPattern[position] = "empty";
                }
            }
            else if (step == 2)
            {
                if (!moving)
                    goToNextPosition(2);
                else if (moving && !indexer.isMoving())
                {
                    moving = false;
                    step++;
                }
            }
            else if (step == 3)
            {
                if (!pulsing) {
                    indexer.handleLeverButton(true);
                    pulsing = true;
                } else if (pulsing && !indexer.getLeverState())
                {
                    pulsing = false;
                    step++;
                    indexerPattern[position] = "empty";
                }
            }
            else if (step == 4)
            {
                if (!moving)
                    goToNextPosition(3);
                else if (moving && !indexer.isMoving())
                {
                    moving = false;
                    step++;
                }
            }
            else if (step == 5)
            {
                if (!pulsing) {
                    indexer.handleLeverButton(true);
                    pulsing = true;
                } else if (pulsing && !indexer.getLeverState())
                {
                    pulsing = false;
                    launching = false;
                    indexerPattern[position] = "empty";
                }
            }
        }
        else
        {
            // if not launching, reset state variables
            launching = false;
            step = 0;
            moving = false;
            pulsing = false;
            position = 0;
        }
    }

    // launchNumber goes 1,2,3
    private void goToNextPosition(int launchNumber)
    {
        moving = true;
        // go to the next position following the pattern
        for (int i=0; i<indexerPattern.length; i++)
        {
            if (wantedPattern[launchNumber-1].equals(indexerPattern[i]))
            {
                indexer.setIndexerPosition("Launch" + (i+1));
                position = i;
                return;
            }
        }

        // go to a position with a ball(if not early return by finding the right ball)
        for (int i=0; i<indexerPattern.length; i++)
        {
            if (indexerPattern[i].equals("purple") || indexerPattern[i].equals("green") || indexerPattern[i].equals("unknown"))
            {
                indexer.setIndexerPosition("Launch" + (i+1));
                position = i;
                return;
            }
        }

        // if no balls to launch, stop launching
        launching = false;
    }

    public void startLaunch(String[] wantedPattern, String[] indexerPattern)
    {
        launching = true;
        this.wantedPattern = wantedPattern;
        this.indexerPattern = indexerPattern;
    }

    public void stopLaunch()
    {
        launching = false;
    }

    public boolean getStatus()
    {
        return launching;
    }

    public String[] getIndexerPattern()
    {
        return indexerPattern;
    }
}