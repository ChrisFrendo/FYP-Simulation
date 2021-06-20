/* Include the controller definition */
#include "basic_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
/* Include e basic loop functions */
#include <loop_functions/basic_loop_functions.h>

/****************************************/
/****************************************/

BasicController::SFoodData::SFoodData() :
        HasFoodItem(false),
        FoodItemIdx(0) {}

void BasicController::SFoodData::Reset() {
    HasFoodItem = false;
    FoodItemIdx = 0;
}

/****************************************/
/****************************************/

BasicController::SDiffusionParams::SDiffusionParams() :
        GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void BasicController::SDiffusionParams::Init(TConfigurationNode &t_node) {
    try {
        CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
        GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
        GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                                 ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
        GetNodeAttribute(t_node, "delta", Delta);
    }
    catch (CARGoSException &ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex)
    }
}

/****************************************/
/****************************************/

void BasicController::SWheelTurningParams::Init(TConfigurationNode &t_node) {
    try {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch (CARGoSException &ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex)
    }
}

/****************************************/
/****************************************/

BasicController::SStateData::SStateData() {}

void BasicController::SStateData::Reset() {
    State = STATE_CHECK_ROLE;
    InCache = false;
    InStoreArea = false;
    InNest = false;
}

/****************************************/
/****************************************/

BasicController::BasicController() :
        m_pcWheels(NULL),
        m_pcLEDs(NULL),
        m_pcProximity(NULL),
        m_pcLight(NULL),
        m_pcGround(NULL) {}

void BasicController::Init(TConfigurationNode &t_node) {
    try {
        /* Initialize sensors/actuators */
        m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
        m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
        m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
        m_pcGround = GetSensor<CCI_GroundSensor>("ground");

        /* Diffusion algorithm */
        m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

        LOG << "Robot init done" << std::endl;
    } catch (CARGoSException &ex) {
        THROW_ARGOSEXCEPTION_NESTED(
                "Error initializing the foot-bot basic controller for robot \"" << GetId() << "\"", ex)
    }

    Reset();
}

void BasicController::ControlStep() {
    switch (m_sStateData.State) {
        case SStateData::STATE_HARVESTING: {
            Harvest();
            break;
        }
        case SStateData::STATE_RETURN_TO_CACHE: {
            ReturnToCache();
            break;
        }
        case SStateData::STATE_CHECK_ROLE: {
            CheckRole();
            break;
        }
        case SStateData::STATE_RETURN_TO_NEST: {
            ReturnToNest();
            break;
        }
        case SStateData::STATE_STORING: {
            Store();
            break;
        }
        default: {
            LOGERR << "We can't be here, there's a bug!" << std::endl;
        }
    }
}

void BasicController::Reset() {
    /* Reset robot state */
    m_sStateData.Reset();
    /* Reset food data */
    m_sFoodData.Reset();
    /* Set LED color */
    m_pcLEDs->SetAllColors(CColor::YELLOW);
}

/****************************************/
/****************************************/

bool realEquals(double a, double b, double epsilon = 0.05) {
    return std::abs(a - b) < epsilon;
}

void BasicController::UpdateState() {
    /* Reset state flags */
    m_sStateData.InCache = false;
    m_sStateData.InStoreArea = false;
    m_sStateData.InNest = false;

    /* Read stuff from the ground and position sensor */
    const std::vector<Real> &tGroundReads = m_pcGround->GetReadings();

    // using ground sensor to determine if we are in storing area or cache area or nest area
    if (tGroundReads[0] == 0.0f && tGroundReads[1] == 0.0f && tGroundReads[2] == 0.0f && tGroundReads[3] == 0.0f) {
        m_sStateData.InCache = true;
    } else if ( realEquals(tGroundReads[0], 0.5f) && realEquals(tGroundReads[1], 0.5f) && realEquals(tGroundReads[2], 0.5f) && realEquals(tGroundReads[3], 0.5f)) {
        m_sStateData.InNest = true;
    } else if ( realEquals(tGroundReads[0], 0.9f) && realEquals(tGroundReads[1], 0.9f) && realEquals(tGroundReads[2], 0.9f) && realEquals(tGroundReads[3], 0.9f)) {
        m_sStateData.InStoreArea = true;
    }

}

/****************************************/
/****************************************/

void BasicController::Harvest() {
    // if the cache is full, the robot should change to check role state
    if (CBasicLoopFunctions::m_currentFoodInCache >= CBasicLoopFunctions::m_maxFoodInCache) {

        if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO2) {
            m_pcLEDs->SetAllColors(CColor::YELLOW);
            m_sStateData.State = SStateData::STATE_CHECK_ROLE;
            return;
        } else if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO3) {
            m_pcWheels->SetLinearVelocity(0, 0);
            return;
        }
    }

    /* We switch to 'return to cache' if we have a food item */
    if (m_sFoodData.HasFoodItem) {
        /* Yes, we do! */
        if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO2 ||
            CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO3) {
            m_pcLEDs->SetAllColors(CColor::BLUE);
            m_sStateData.State = SStateData::STATE_RETURN_TO_CACHE;
        } else if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO1) {
            m_pcLEDs->SetAllColors(CColor::ORANGE);
            m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
        }
    } else {
        /* Perform the actual exploration */
        UpdateState();

        /* Get the diffusion vector to perform obstacle avoidance */
        bool bCollision;
        CVector2 cDiffusion = DiffusionVector(bCollision);

        /* If we are anywhere outside harvest area, then use inverse of light vector to move towards harvest area */
        if (m_sStateData.InCache || m_sStateData.InStoreArea || m_sStateData.InNest) {
            SetWheelSpeedsFromVector(
                    m_sWheelTurningParams.MaxSpeed * cDiffusion - (m_sWheelTurningParams.MaxSpeed * 0.75f * CalculateVectorToLight()));
        } else {
            /* Use the diffusion vector only */
            SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
        }
    }
}

/****************************************/
/****************************************/

void BasicController::ReturnToCache() {
    /* As soon as you get to the cache, switch to 'resting' */
    UpdateState();

    /* Are we in the cache? */
    if (m_sStateData.InCache) {
        m_pcLEDs->SetAllColors(CColor::YELLOW);
        m_sStateData.State = SStateData::STATE_CHECK_ROLE;
    } else {
        /* Still outside the nest */
        bool bCollision;
        SetWheelSpeedsFromVector(
                m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
                m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
    }

}

void BasicController::CheckRole() {

    switch (CBasicLoopFunctions::m_selectedAlgo) {
        case CBasicLoopFunctions::ALGO2:
            if (CBasicLoopFunctions::m_currentFoodInCache < CBasicLoopFunctions::m_maxFoodInCache) {
                // switch to STATE_HARVESTING
                m_pcLEDs->SetAllColors(CColor::GREEN);
                m_sStateData.State = SStateData::STATE_HARVESTING;
            } else {
                // switch to STATE_RETURN_TO_NEST for now
                m_pcLEDs->SetAllColors(CColor::ORANGE);
                m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
            }
            break;
        case CBasicLoopFunctions::ALGO3:
            m_pcLEDs->SetAllColors(CColor::BLACK);
            m_sStateData.State = m_sStateData.originalState;
            break;
        default:
            LOGERR << "Invalid algorithm when in CheckRole(), should never happen" << std::endl;
    }

}

void BasicController::ReturnToNest() {
    UpdateState();

    // checking if in nest area
    if (m_sStateData.InNest) {

        if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO2 ||
            CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO3) {
            /* Once in the nest, go back to STORING STATE */
            m_pcLEDs->SetAllColors(CColor::MAGENTA);
            m_sStateData.State = SStateData::STATE_STORING;
        } else if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO1) {
            m_pcLEDs->SetAllColors(CColor::GREEN);
            m_sStateData.State = SStateData::STATE_HARVESTING;
        }
        return;
    }

    /* Keep going towards nest area */
    bool bCollision;
    SetWheelSpeedsFromVector(
            m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
            m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}

void BasicController::Store() {

    // getting new state
    UpdateState();

    // if the current robot does not have a food item AND the cache is empty, then check role
    if (CBasicLoopFunctions::m_selectedAlgo == CBasicLoopFunctions::ALGO2 &&
        CBasicLoopFunctions::m_currentFoodInCache <= 0 && !m_sFoodData.HasFoodItem) {
        m_pcLEDs->SetAllColors(CColor::YELLOW);
        m_sStateData.State = SStateData::STATE_CHECK_ROLE;
        return;
    }

    if (m_sStateData.InCache) {
        // we have reached the cache, so we should stop moving, and change state to RETURN_TO_NEST
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);

        if (!m_sFoodData.HasFoodItem) return;
//        while (CBasicLoopFunctions::m_currentFoodInCache <= 0);

        m_pcLEDs->SetAllColors(CColor::ORANGE);
        m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
    } else {
        // if we are not in the cache, we need to keep moving using the diffusion vector and the negative of the light vector
        bool bCollision;
        SetWheelSpeedsFromVector(
                m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) -
                m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
    }
}

/****************************************/
/****************************************/

CVector2 BasicController::CalculateVectorToLight() {
    /* Get readings from light sensor */
    const CCI_FootBotLightSensor::TReadings &tLightReads = m_pcLight->GetReadings();

    /* Sum them together */
    CVector2 cAccumulator;
    for (size_t i = 0; i < tLightReads.size(); ++i) {
        cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
    }

    /* If the light was perceived, return the vector */
    if (cAccumulator.Length() > 0.0f) {
        return CVector2(1.0f, cAccumulator.Angle());
    } else {
        return CVector2();
    }
}

/****************************************/
/****************************************/

CVector2 BasicController::DiffusionVector(bool &b_collision) {
    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();

    /* Sum them together */
    CVector2 cDiffusionVector;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }

    /* If the angle of the vector is small enough and the closest obstacle
       is far enough, ignore the vector and go straight, otherwise return
       it */
    if (m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
        cDiffusionVector.Length() < m_sDiffusionParams.Delta) {
        b_collision = false;
        return CVector2::X;
    } else {
        b_collision = true;
        cDiffusionVector.Normalize();
        return -cDiffusionVector;
    }
}

/****************************************/
/****************************************/

void BasicController::SetWheelSpeedsFromVector(const CVector2 &c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    /* State transition logic */
    if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
        if (Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
        if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        } else if (Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
        }
    }
    if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
        if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        } else if (Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch (m_sWheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                                m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 = m_sWheelTurningParams.MaxSpeed;
            break;
        }
    }
    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if (cHeadingAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    } else {
        /* Turn Right */
        fLeftWheelSpeed = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

REGISTER_CONTROLLER(BasicController, "basic_controller")

