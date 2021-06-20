#include "basic_qtuser_functions.h"
#include "basic_loop_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
    RegisterUserFunction<CIDQTUserFunctions, CFloorEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CFloorEntity &c_entity) {
    // used to draw text on simulation to show current cache and nest storage status

    char msg[10];
    char msg2[10];

    sprintf(msg, "%d/%d", CBasicLoopFunctions::m_currentFoodInCache, CBasicLoopFunctions::m_maxFoodInCache);
    sprintf(msg2, "%d", CBasicLoopFunctions::m_currentFoodInNest);

    DrawText(CVector3(1.2, 0.0, 0.5), msg);
    DrawText(CVector3(1.2, -2.0, 0.5), msg2);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "basic_qtuser_functions")
