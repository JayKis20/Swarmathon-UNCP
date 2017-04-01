#include "SearchController.h"

//--------------------------------------
geometry_msgs::Pose2D CenterLocation;


//BOOLEAN TRIGGERS
//--------------------------------------
bool CenterSeen;
bool first;

//PRIMITIVES
//--------------------------------------

//Target Search
int searchLoop;             //int of 0 - 8
int looping;                //int of 0 - 7

double searchDist;
double searchCounter;

//Center Search
int centerLoop;

double centerSearch;

SearchController::SearchController()
{

    // VARIABLES

    searchLoop = 0;
    looping = 0;
    searchDist = .3;

    centerLoop = 0;
    centerSearch = .4;

    NumRotations = 0;

    CenterSeen = false;
    first = true;

    rng = new random_numbers::RandomNumberGenerator();

    //searchLoop = rng->uniformInteger(0, 8);                 //random point in loop between 0 and 8
    searchCounter = rng->uniformReal(.5, 1.00);    //random distance from center to start searching

    centerLoop = rng->uniformInteger(0, 8);
    searchDist = .3;

    CenterLocation.x = 0;
    CenterLocation.y = 0;
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation)
{
  geometry_msgs::Pose2D goalLocation;

       //TargetSearch(currentLocation, goalLocation, CenterLocation);
        if (searchLoop < 0 || searchLoop > 8)
        {
            searchLoop = 0;
            NumRotations++;
        }

        if (searchLoop == 0)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x + searchCounter;
            goalLocation.y = CenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            searchCounter += searchDist;
            NumRotations++;
        }
        else if (searchLoop == 1)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x + searchCounter;
            goalLocation.y = CenterLocation.y + searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            searchCounter += searchDist;
            NumRotations++;
        }
        else if (searchLoop == 2)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x + searchCounter / 2;
            goalLocation.y = CenterLocation.y + searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            searchCounter += searchDist;
            NumRotations++;
        }
        else if (searchLoop == 3)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x - searchCounter / 2;
            goalLocation.y = CenterLocation.y + searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
        }
        else if (searchLoop == 4)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x - searchCounter;
            goalLocation.y = CenterLocation.y + searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
            looping++;

            searchCounter += searchDist;
            NumRotations++;
        }
        else if (searchLoop == 5)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x - searchCounter;
            goalLocation.y = CenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));
        }
        else if (searchLoop == 6)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x - searchCounter / 2;
            goalLocation.y = CenterLocation.y - searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            searchCounter += searchDist;
            NumRotations++;
        }
        else if (searchLoop == 7)
        {
            searchLoop++;
            goalLocation.x = CenterLocation.x + searchCounter / 2;
            goalLocation.y = CenterLocation.y - searchCounter;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            searchCounter += searchDist;
            NumRotations++;
        }
        else if (searchLoop == 8)
        {
            goalLocation.x = CenterLocation.x + searchCounter;
            goalLocation.y = CenterLocation.y - searchCounter / 2;
            goalLocation.theta = atan2((goalLocation.y - currentLocation.y), (goalLocation.x - currentLocation.x));

            searchCounter += searchDist;
            NumRotations++;

            //if they do not complete at least 4 full rotations
            if(NumRotations >= 4)
            {
                searchCounter = searchCounter + searchDist; //?

                NumRotations = 0;
            }
            else if(NumRotations > 2)
            {
                NumRotations = 0;
            }

            searchLoop = 0;
        }

    return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation)
{

    if (currentLocation.theta <= 45 * M_PI / 180)
    {
        searchLoop = 2;
    }
    else if (currentLocation.theta <= 90 * M_PI / 180)
    {
        searchLoop = 4;
    }
    else if (currentLocation.theta <= 135 * M_PI / 180)
    {
        searchLoop = 5;
    }
    else if (currentLocation.theta <= 180 * M_PI / 180)
    {
        searchLoop = 7;
    }
    else if (currentLocation.theta <= 225 * M_PI / 180)
    {
        searchLoop = 7;
    }
    else if (currentLocation.theta <= 270 * M_PI / 180)
    {
        searchLoop = 0;
    }
    else if (currentLocation.theta <= 310 * M_PI / 180)
    {
        searchLoop = 1;
    }
    else if (currentLocation.theta <= 360 * M_PI / 180)
    {
        searchLoop = 2;
    }

    geometry_msgs::Pose2D newGoalLocation;

    //remainingGoalDist avoids magic numbers by calculating the dist
    // double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

    newGoalLocation = search(currentLocation);

    //this of course assumes random walk continuation. Change for diffrent search methods.
    newGoalLocation.theta = oldGoalLocation.theta;
    newGoalLocation.x = currentLocation.x + (0.20 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
    newGoalLocation.y = currentLocation.y + (0.20 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

    return newGoalLocation;
}

void SearchController::setCenterSeen(bool answer)
{
    CenterSeen = answer;
}

void SearchController::setCenterLocation(geometry_msgs::Pose2D newLocation)
{
    CenterLocation = newLocation;
}

void SearchController::SetRotations(int num)
{
    NumRotations = num;
}


