#ifndef SIMULATION_BASIC_CONTROLLER_H
#define SIMULATION_BASIC_CONTROLLER_H

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class BasicController : public CCI_Controller {

public:

    /*
     * This structure holds data about food collecting by the robots
     */
    struct SFoodData {
        bool HasFoodItem;      // true when the robot is carrying a food item
        size_t FoodItemIdx;    // the index of the current food item in the array of available food items

        SFoodData();

        void Reset();
    };

    /*
     * The following variables are used as parameters for the
     * diffusion algorithm.
     */
    struct SDiffusionParams {
        /*
         * Maximum tolerance for the proximity reading between
         * the robot and the closest obstacle.
         * The proximity reading is 0 when nothing is detected
         * and grows exponentially to 1 when the obstacle is
         * touching the robot.
         */
        Real Delta;
        /* Angle tolerance range to go straight. */
        CRange<CRadians> GoStraightAngleRange;

        /* Constructor */
        SDiffusionParams();

        /* Parses the XML section for diffusion */
        void Init(TConfigurationNode &t_tree);
    };

    /*
     * The following variables are used as parameters for
     * turning during navigation.
     */
    struct SWheelTurningParams {
        /*
         * The turning mechanism.
         * The robot can be in three different turning states.
         */
        enum ETurningMechanism {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
         * Angular thresholds to change turning state.
         */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode &t_tree);
    };

    /*
     * Contains all the state information about the controller.
     */
    struct SStateData {
        /* The possible states in which the controller can be */
        enum EState {
            STATE_HARVESTING = 0,
            STATE_RETURN_TO_CACHE,
            STATE_CHECK_ROLE,
            STATE_RETURN_TO_NEST,
            STATE_STORING
        } State;

        /* True when the robot is in the cache site (black area on floor) */
        bool InCache;

        /* True when the robot is in the storing area (right side of the cache) */
        bool InStoreArea;

        /* True when robot is in the nest area (gray area on floor) */
        bool InNest;

        EState originalState{STATE_HARVESTING};

        SStateData();

        void Reset();
    };

public:

    /* Class constructor. */
    BasicController();

    /* Class destructor. */
    ~BasicController() override = default;

    /*
     * This function initializes the controller.
     */
    void Init(TConfigurationNode &t_node) override;

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    void ControlStep() override;

    /*
     * This function resets the controller to its original state right after the
     * It is called when you press the reset button in the GUI.
     */
    void Reset() override;

    inline bool IsHarvesting() const {
        return m_sStateData.State == SStateData::STATE_HARVESTING;
    }

    inline bool IsReturningToCache() const {
        return m_sStateData.State == SStateData::STATE_RETURN_TO_CACHE;
    }

    inline bool IsReturningToNest() const {
        return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
    }

    inline bool IsStoring() const {
        return m_sStateData.State == SStateData::STATE_STORING;
    }

    inline SFoodData &GetFoodData() {
        return m_sFoodData;
    }

    inline SStateData &GetStateData() {
        return m_sStateData;
    }

private:

    /*
     * Updates the state information.
     */
    void UpdateState();

    /*
     * Calculates the vector to the light.
     */
    CVector2 CalculateVectorToLight();

    /*
     * Calculates the diffusion vector. If there is a close obstacle,
     * it points away from it; it there is none, it points forwards.
     * The b_collision parameter is used to return true or false whether
     * a collision avoidance just happened or not. It is necessary for the
     * collision rule.
     */
    CVector2 DiffusionVector(bool &b_collision);

    /*
     * Gets a direction vector as input and transforms it into wheel
     * actuation.
     */
    void SetWheelSpeedsFromVector(const CVector2 &c_heading);

    /*
     * Executes the return to nest state.
     */
    void ReturnToNest();

    /*
     * Executes the exploring state.
     */
    void Harvest();

    /*
     * Executes the return to cache state.
     */
    void ReturnToCache();

    /*
     * Executes the check role state
     */
    void CheckRole();

    /*
     * Executes the storing state.
     */
    void Store();

private:

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator *m_pcWheels;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator *m_pcLEDs;
    /* Pointer to the foot-bot proximity sensor */
    CCI_FootBotProximitySensor *m_pcProximity;
    /* Pointer to the foot-bot light sensor */
    CCI_FootBotLightSensor *m_pcLight;
    /* Pointer to the generic ground sensor */
    CCI_GroundSensor *m_pcGround;

    /* The controller state information */
    SStateData m_sStateData;
    /* The turning parameters */
    SWheelTurningParams m_sWheelTurningParams;
    /* The diffusion parameters */
    SDiffusionParams m_sDiffusionParams;
    /* The food data */
    SFoodData m_sFoodData;
};

#endif //SIMULATION_BASIC_CONTROLLER_H
