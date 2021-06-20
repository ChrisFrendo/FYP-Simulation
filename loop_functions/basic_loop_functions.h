#ifndef BASIC_LOOP_FUNCTIONS_H
#define BASIC_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CBasicLoopFunctions : public CLoopFunctions {

public:

    CBasicLoopFunctions();

    ~CBasicLoopFunctions() override = default;

    void Init(TConfigurationNode &t_tree) override;

    void Reset() override;

    void Destroy() override;

    CColor GetFloorColor(const CVector2 &c_position_on_plane) override;

    void PreStep() override;

    void PostExperiment() override;

    static UInt32 m_currentFoodInCache; // holds the current number of food items in the cache
    static UInt32 m_maxFoodInCache; // holds the maximum number of food items in the cache
    static UInt32 m_currentFoodInNest; // holds the current number of food items stored in the nest

    enum ALGORITHM {
        ALGO1 = 0,
        ALGO2,
        ALGO3,
        INVALID
    };

    static ALGORITHM m_selectedAlgo;

private:

    Real m_fFoodSquareRadius;
    CRange<Real> m_cHarvestingArenaSideX, m_cHarvestingArenaSideY;
    std::vector<CVector2> m_cFoodPos;
    CFloorEntity *m_pcFloor;
    CRandom::CRNG *m_pcRNG;

    std::string m_strOutput;
    std::ofstream m_cOutput;

    UInt32 m_totalCachedFood; // holds total number of food placed in the cache
};

#endif