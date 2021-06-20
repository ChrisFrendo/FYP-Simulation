#include "basic_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/basic_controller.h>

/****************************************/
/****************************************/

UInt32 CBasicLoopFunctions::m_currentFoodInCache = 0;
UInt32 CBasicLoopFunctions::m_maxFoodInCache = 0;
UInt32 CBasicLoopFunctions::m_currentFoodInNest = 0;
CBasicLoopFunctions::ALGORITHM CBasicLoopFunctions::m_selectedAlgo = INVALID;

CBasicLoopFunctions::CBasicLoopFunctions() :
        m_cHarvestingArenaSideX(-1.0f, 1.0f),
        m_cHarvestingArenaSideY(-1.0f, 1.0f),
        m_pcFloor(NULL),
        m_pcRNG(NULL),
        m_totalCachedFood(0) {
}

/****************************************/
/****************************************/

void CBasicLoopFunctions::Init(TConfigurationNode &t_node) {
    try {
        TConfigurationNode &tHarvesting = GetNode(t_node, "harvesting");
        TConfigurationNode &tCache = GetNode(t_node, "cache");
        TConfigurationNode &tAlgorithm = GetNode(t_node, "algorithm");

        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        /* Get the number of food items we want to be scattered from XML */
        UInt32 unFoodItems;
        GetNodeAttribute(tHarvesting, "items", unFoodItems);

        /* Getting min and max x/y values for harvesting area */
        Real minX, maxX, minY, maxY;
        GetNodeAttribute(tHarvesting, "min_x", minX);
        GetNodeAttribute(tHarvesting, "max_x", maxX);
        GetNodeAttribute(tHarvesting, "min_y", minY);
        GetNodeAttribute(tHarvesting, "max_y", maxY);

        m_cHarvestingArenaSideX = CRange<Real>(minX, maxX);
        m_cHarvestingArenaSideY = CRange<Real>(minY, maxY);

        /* Get the radius of each food item from XML */
        GetNodeAttribute(tHarvesting, "radius", m_fFoodSquareRadius);
        m_fFoodSquareRadius *= m_fFoodSquareRadius;
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        /* Distribute uniformly the items in the environment */
        for (UInt32 i = 0; i < unFoodItems; ++i) {
            m_cFoodPos.push_back(
                    CVector2(m_pcRNG->Uniform(m_cHarvestingArenaSideX),
                             m_pcRNG->Uniform(m_cHarvestingArenaSideY)));
        }
        /* Get the output file name from XML */
        GetNodeAttribute(tHarvesting, "output", m_strOutput);
        /* Open the file, erasing its contents */
        m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
        m_cOutput << "clock,harvesting,storing,current_cache,total_cached,total_harvested" << std::endl;

        // getting max number of food items that can be stored in the cache
        GetNodeAttribute(tCache, "maxFoodCache", CBasicLoopFunctions::m_maxFoodInCache);

        // determining selected algorithm
        // getting algorithm option
        std::string algoInput;
        GetNodeAttribute(tAlgorithm, "algorithm", algoInput);

        CSpace::TMapPerType &m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

        if (algoInput.compare("algo1") == 0) {
            CBasicLoopFunctions::m_selectedAlgo = ALGO1;

            for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
                CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
                BasicController &cController = dynamic_cast<BasicController &>(cFootBot.GetControllableEntity().GetController());

                cController.GetActuator<CCI_LEDsActuator>("leds")->SetAllColors(CColor::GREEN);
                cController.GetStateData().originalState = BasicController::SStateData::STATE_HARVESTING;
                cController.GetStateData().State = BasicController::SStateData::STATE_HARVESTING;

            }

        } else if (algoInput.compare("algo2") == 0) {
            CBasicLoopFunctions::m_selectedAlgo = ALGO2;
        } else if (algoInput.compare("algo3") == 0) {
            CBasicLoopFunctions::m_selectedAlgo = ALGO3;

            UInt32 numHarvesters;
            UInt32 numStorers;
            GetNodeAttribute(tAlgorithm, "num_harvesters", numHarvesters);
            GetNodeAttribute(tAlgorithm, "num_storers", numStorers);

            if (numHarvesters + numStorers != m_cFootbots.size()) {
                throw CARGoSException("Number of harvesters and storers does not match total number of robots");
            }

            int i = 0;
            for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
                CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
                BasicController &cController = dynamic_cast<BasicController &>(cFootBot.GetControllableEntity().GetController());

                if (i < numHarvesters) {
                    cController.GetActuator<CCI_LEDsActuator>("leds")->SetAllColors(CColor::CYAN);
                    cController.GetStateData().originalState = BasicController::SStateData::STATE_HARVESTING;
                    cController.GetStateData().State = BasicController::SStateData::STATE_HARVESTING;
                } else {
                    cController.GetActuator<CCI_LEDsActuator>("leds")->SetAllColors(CColor::PURPLE);
                    cController.GetStateData().originalState = BasicController::SStateData::STATE_STORING;
                    cController.GetStateData().State = BasicController::SStateData::STATE_STORING;
                }
                i++;
            }
        } else {
            throw CARGoSException("Invalid algorithm attribute, must be: <algo1|algo2>");
        }

        LOG << "loop init done" << std::endl;
    }
    catch (CARGoSException &ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex)
    }
}

/****************************************/
/****************************************/

void CBasicLoopFunctions::Reset() {
    /* Zero the counters */
    m_totalCachedFood = 0;
    m_currentFoodInCache = 0;

    /* Close the file */
    m_cOutput.close();
    /* Open the file, erasing its contents */
    m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
    m_cOutput << "clock,harvesting,storing,current_cache,total_cached,total_harvested" << std::endl;

    /* Distribute uniformly the items in the environment */
    for (UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
        m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cHarvestingArenaSideX),
                          m_pcRNG->Uniform(m_cHarvestingArenaSideY));
    }
}

/****************************************/
/****************************************/

void CBasicLoopFunctions::Destroy() {
    /* Close the file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CBasicLoopFunctions::GetFloorColor(const CVector2 &c_position_on_plane) {

    if ((c_position_on_plane.GetY() < -1.95f && c_position_on_plane.GetY() > -2.25f) &&
        c_position_on_plane.GetX() < 1.0f && c_position_on_plane.GetX() > -1.0f) {
        return CColor::GRAY50;
    } else if (c_position_on_plane.GetY() < 0.25f && c_position_on_plane.GetY() > -0.25f &&
               c_position_on_plane.GetX() < 1.0f && c_position_on_plane.GetX() > -1.0f) {
        return CColor::BLACK;
    } else if ((c_position_on_plane.GetY() <= -0.25 && c_position_on_plane.GetY() >= -1.95f) &&
               c_position_on_plane.GetX() < 1.0f && c_position_on_plane.GetX() > -1.0f) {
        return CColor::GRAY90;
    }

    for (UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
        if ((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
            return CColor::BLACK;
        }
    }

    return CColor::WHITE;
}

/****************************************/
/****************************************/

bool isInCache(CVector2 pos) {
    return pos.GetY() > -0.25f && pos.GetY() < 0.25f && pos.GetX() > -1.00f && pos.GetX() < 1.00f;
}

bool isInNest(CVector2 pos) {
    return pos.GetY() < -1.95f && pos.GetY() > -2.25f && pos.GetX() < 1.0f && pos.GetX() > -1.0;
}

/****************************************/
/****************************************/

void CBasicLoopFunctions::PreStep() {

    UInt32 numHarvesting = 0;
    UInt32 numStoring = 0;
    /* Check whether a robot is on a food item */
    CSpace::TMapPerType &m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

    for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
        BasicController &cController = dynamic_cast<BasicController &>(cFootBot.GetControllableEntity().GetController());

        /* Count how many foot-bots are in which state */
        if (cController.IsHarvesting() || cController.IsReturningToCache()) {
            ++numHarvesting;
        } else if (cController.IsReturningToNest() || cController.IsStoring()) {
            ++numStoring;
        }

        /* Get the position of the foot-bot on the ground as a CVector2 */
        CVector2 cPos;
        cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        /* Get food data */
        BasicController::SFoodData &sFoodData = cController.GetFoodData();

        /* The foot-bot has a food item */
        if (sFoodData.HasFoodItem) {
            /* Check whether the foot-bot is in the cache area */
            if (isInCache(cPos)) {

                // the robot has a food item and is in the cache.
                if (cController.IsHarvesting() || cController.IsReturningToCache()) {
                    /* Place a new food item on the ground */
                    m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cHarvestingArenaSideX),
                                                          m_pcRNG->Uniform(m_cHarvestingArenaSideY));
                    /* Drop the food item */
                    sFoodData.HasFoodItem = false;
                    sFoodData.FoodItemIdx = 0;

                    if (CBasicLoopFunctions::m_currentFoodInCache < m_maxFoodInCache) {
                        // add food to cache iff it is not full, else discard it
                        ++m_totalCachedFood;
                        ++CBasicLoopFunctions::m_currentFoodInCache;
                    }

                    /* The floor texture must be updated */
                    m_pcFloor->SetChanged();
                }
            } else if (isInNest(cPos)) {
                if (cController.IsReturningToNest()) {
                    sFoodData.HasFoodItem = false;
                    ++CBasicLoopFunctions::m_currentFoodInNest;

                    if (CBasicLoopFunctions::m_selectedAlgo == ALGO1) {
                        m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cHarvestingArenaSideX),
                                                              m_pcRNG->Uniform(m_cHarvestingArenaSideY));
                        sFoodData.FoodItemIdx = 0;

                        m_pcFloor->SetChanged();
                    }
                }
            }
        } else {
            /* The foot-bot has no food item */
            /* Check whether the foot-bot is out of the cache area */
            if (!isInCache(cPos)) {
                /* Check whether the foot-bot is on a food item */

                if (cController.IsHarvesting()) {
                    bool bDone = false;
                    for (size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
                        if ((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {

                            /* If so, we move that item out of sight */
                            m_cFoodPos[i].Set(100.0f, 100.f);

                            /* The foot-bot is now carrying an item */
                            sFoodData.HasFoodItem = true;
                            sFoodData.FoodItemIdx = i;

                            /* The floor texture must be updated */
                            m_pcFloor->SetChanged();
                            /* We are done */
                            bDone = true;
                        }
                    }
                }
            } else {
                // robot does NOT have a food item, but he is in the cache.

                if (cController.IsStoring()) {

                    // remove food from cache iff there is some food in the cache
                    if (CBasicLoopFunctions::m_currentFoodInCache > 0) {
                        --CBasicLoopFunctions::m_currentFoodInCache;
                        sFoodData.HasFoodItem = true;
                    }
                }
            }
        }
    }

    /* Output stuff to file */
    m_cOutput << GetSpace().GetSimulationClock() << ","
              << numHarvesting << ","
              << numStoring << ","
              << CBasicLoopFunctions::m_currentFoodInCache << ","
              << CBasicLoopFunctions::m_totalCachedFood << ","
              << CBasicLoopFunctions::m_currentFoodInNest << std::endl;
}

void CBasicLoopFunctions::PostExperiment() {
    m_cOutput.close();
    exit(EXIT_SUCCESS);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBasicLoopFunctions, "basic_loop_functions")
