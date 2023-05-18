#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations

namespace tf2 {

    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
        double x_pos = rangeBearing.range * sin(-rangeBearing.bearing+ tf::quaternionToYaw(aircraft.orientation))+aircraft.position.x;
        double y_pos = rangeBearing.range * cos(-rangeBearing.bearing+ tf::quaternionToYaw(aircraft.orientation))+aircraft.position.y;
        Point p;
        p.x = x_pos;
        p.y = y_pos;
        p.z = aircraft.position.z;
        return p;

    }

    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft)
    {
        RangeBearingStamped rbstamped = {0, 0,0};

        double x_pos = globalEnemy.x - aircraft.position.x;
        double y_pos = globalEnemy.y - aircraft.position.y;
        double theta = normaliseAngle(atan2(y_pos,x_pos));
        rbstamped.range = sqrt(pow(x_pos,2)+pow(y_pos,2));
        double angle = normaliseAngle(theta - tf::quaternionToYaw(aircraft.orientation));

        rbstamped.bearing = angle;
        return rbstamped;
    }


    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      if (theta > M_PI){
          theta = -( (2* M_PI) - theta);
      }

      return theta;
    }

}
