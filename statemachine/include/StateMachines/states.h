#ifndef SUBSTATEMACHINESTATES_H
#define SUBSTATEMACHINESTATES_H


enum class GeneralStates {
    START,
    TURN_TO_NO_BLOB,
    FIND_COLOR,
    CALC_A_B,
    SHOOT_PUCK,
    WAIT_A_B,
    LOCALIZE,
    GO_TO_MIDDLE_POS,
    SEARCH_NEAREST_PUCK,
    RECOVERY,
    END
};

enum class FindColorStates {
    START,
    TURN_WITHOUT_DRIVE,
    DRIVE_ARROUND,
    DRIVE_ARROUND2,
    DRIVE_TO_MIDDLE,
    DRIVE_TO_INIT,
    DRIVE_TO_LEFT,
    DRIVE_TO_RIGHT,
    TURN_TO_PUCKS,
    FIND_COLOR,
    END
};

enum class Localize {
    START,
    TURN_TO_3_GREEN,
    SEND_A_B,
    WAIT,
    END
};

enum class ShootStates {
    START,
    DETECTED_ROBOT,
    TURN,
    FIND_PUCK,
    FIND_PUCK_EX,
    FIND_PUCK_DETAIL,
    FIND_RECALIBRATE,
    DRIVE_FORWARD,
    DRIVE_TO_PUCK,
    DRIVE_TO_PUCK_CLOSE,
    DRIVE_TO_GOAL,
    DROP_PUCK,
    DRIVE_TO_BACK_HOME,
    DRIVE_LITTLE_BACK,
    DRIVE_RANDOM,
    END
};

enum class ObjectParserStates {
    START,
    GET_OBJECTS,
    PARSING_OBJECTS,
    DECREASE_PROBABILITY_COUNTER,
    END
};

enum class CalculateAB_States {
    START,
    TURN,
    CALC_BLOBS,
    TURN_TO_NO_GREEN,
    TURN_TO_3_GREEN_LEFT,
    TURN_TO_3_GREEN_RIGHT,
    A,
    B,
    CALCULATE_SIDE,
    END
};

enum class RecoverStates {
    START,
    RECONNECT,
    STOP,
    END
};


#endif // SUBSTATEMACHINESTATES_H

