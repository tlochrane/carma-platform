package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.GuidanceCommands;

/**
 * Represents a longitudinal maneuver in which the vehicle steadily increases its speed.
 */
public class SpeedUp extends LongitudinalManeuver {

    /**
     * ASSUMES that the target speed has been specified such that it does not exceed and infrastructure speed limit.
     */
    @Override
    public void plan(IManeuverInputs inputs, GuidanceCommands commands, double startDist) throws IllegalStateException {
        super.plan(inputs, commands, startDist);

        //if speed change is going to be only slight then
            //cut the acceleration rate to half the limit

        //compute the distance to be covered during a linear (in time) speed change, assuming perfect vehicle response

        //add the distance covered by the expected vehicle lag

    }


    @Override
    public void executeTimeStep() {

        //if current location along route < maneuver start distance then
            //set command to start speed

        //if current location along route > maneuver end distance then
            //set command to target speed

        //else if current location is > start distances then
            //compute command based on linear interpolation on time steps

        //invoke the ACC override

        //send the command to the vehicle

    }
}
