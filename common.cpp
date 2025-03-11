#include "common.h"

// Global variable definitions:
int verbose = 0;
double begin_time = 0.0;
double timeout = 0.0;

VAR_DEC_HEURISTIC VarDecHeuristic = VAR_DEC_HEURISTIC::MINISAT;
VAL_DEC_HEURISTIC ValDecHeuristic = VAL_DEC_HEURISTIC::PHASESAVING;

unordered_map<string, option*> options = {
    {"v", new intoption(&verbose, 0, 2, "Verbosity level")},
    {"timeout", new doubleoption(&timeout, 0.0, 36000.0, "Timeout in seconds")},
    {"valdh", new intoption((int*)&ValDecHeuristic, 0, 1, "{0: phase-saving, 1: literal-score}")}
};
