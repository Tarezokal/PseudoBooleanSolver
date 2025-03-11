#pragma once

#include <iostream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <utility>
#include "options.h"

using namespace std;

//---------------------------
// Common typedefs and macros
//---------------------------
typedef int Var;
typedef int Lit;
typedef vector<Lit> clause_t;
typedef clause_t::iterator clause_it;
typedef vector<Lit> trail_t;

#define Assert(exp) AssertCheck(exp, __func__, __LINE__)
#define Neg(l) (l & 1)
#define Restart_multiplier 1.1f
#define Restart_lower 100
#define Restart_upper 1000
#define Max_bring_forward 10
#define var_decay 0.99
#define Rescale_threshold 1e100
#define Assignment_file "assignment.txt"

//---------------------------
// Globals (declared as extern)
//---------------------------
extern int verbose;
extern double begin_time;
extern double timeout;

//---------------------------
// Function declarations
//---------------------------
void Abort(string s, int i);

//---------------------------
// Enums and global variables for decision heuristics
//---------------------------
enum class VAR_DEC_HEURISTIC {
    MINISAT
    // add other decision heuristics here if needed.
};
extern VAR_DEC_HEURISTIC VarDecHeuristic;

enum class VAL_DEC_HEURISTIC {
    PHASESAVING,  // Same as last value; initially false
    LITSCORE      // Choose literal with highest frequency
};
extern VAL_DEC_HEURISTIC ValDecHeuristic;

// Options map for command-line parameters
extern unordered_map<string, option*> options;

//---------------------------
// Other common enums
//---------------------------
enum class LitState {
    L_UNSAT,
    L_SAT,
    L_UNASSIGNED
};

enum class VarState {
    V_FALSE,
    V_TRUE,
    V_UNASSIGNED
};

enum class ClauseState {
    C_UNSAT,
    C_SAT,
    C_UNIT,
    C_UNDEF
};

enum class SolverState {
    UNSAT,
    SAT,
    CONFLICT,
    UNDEF,
    TIMEOUT
};

//---------------------------
// Inline utility functions
//---------------------------
#ifdef _MSC_VER
static inline double cpuTime(void) {
    return (double)clock() / CLOCKS_PER_SEC;
}
#else
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
static inline double cpuTime(void) {
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1000000;
}
#endif

static inline void AssertCheck(bool cond, string func_name, int line, string msg = "") {
    if (cond) return;
    cout << "Assertion fail" << endl;
    cout << msg << endl;
    cout << func_name << " line " << line << endl;
    exit(1);
}

static inline bool match(ifstream& in, char* str) {
    for (; *str != '\0'; ++str)
        if (*str != in.get())
            return false;
    return true;
}

static inline unsigned int Abs(int x) {
    return x < 0 ? (unsigned int)-x : (unsigned int)x;
}

static inline unsigned int v2l(int i) {
    return (i < 0) ? (((-i) << 1) - 1) : (i << 1);
}

static inline Var l2v(Lit l) {
    return (l + 1) / 2;
}

static inline Lit negate(Lit l) {
    return (Neg(l)) ? l + 1 : l - 1;
}

static inline int l2rl(int l) {
    return (Neg(l)) ? -((l + 1) / 2) : l / 2;
}
