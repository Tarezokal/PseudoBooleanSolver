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
#include "options.h"
#include "common.h"

// PB-specific enum for pseudo-Boolean constraints:
enum class PBConstraintState {
    PB_UNDEF,
    PB_SAT,
    PB_UNIT,
    PB_UNSAT
};

//-------------------------------------------------------------
// Constraint class for pseudo-Boolean constraints
//-------------------------------------------------------------
class Constraint {
    std::vector<std::pair<int, int>> terms;  // (coefficient, variable)
    clause_t literals;                       // corresponding literals
    std::vector<int> coefficients;           // coefficients
    int rhs;
    int lhs;
    int lw, rw;                              // watched literal indices
    std::unordered_set<int> watch_set;
    bool flag_new;
public:
    Constraint() : lhs(0) {}

    void insert_term(int coeff, Lit var) {
        terms.emplace_back(coeff, var);
        literals.push_back(var);
        coefficients.push_back(coeff);
    }

    void print_real_lits() {
        Lit l;
        std::cout << "(";
        for (clause_it it = literals.begin(); it != literals.end(); ++it) {
            l = l2rl(*it);
            std::cout << l << " ";
        }
        std::cout << ")";
    }

    void set_watches_set(std::unordered_set<int> watches) { watch_set = watches; }
    void lw_set(int i) { lw = i; }
    void rw_set(int i) { rw = i; }
    std::vector<std::pair<int, int>> get_terms() { return terms; }
    clause_t& get_literals() { return literals; }
    std::vector<int>& get_coefficients() { return coefficients; }
    void reset_literals() { literals.clear(); }
    void reset_coefficients() { coefficients.clear(); }
    void reset_terms() { terms.clear(); }
    void set_flag_new(bool b) { flag_new = b; }
    bool get_flag_new() { return flag_new; }
    int get_lw() { return lw; }
    std::unordered_set<int> get_watch_set() { return watch_set; }
    int get_rw() { return rw; }
    void set_rhs(int v) { rhs = v; }
    int get_rhs() { return rhs; }
    void set_lhs(int v) { lhs = v; }
    int get_lhs() { return lhs; }
    int get_lw_lit() const { return literals[lw]; }
    int get_rw_lit() const { return literals[rw]; }
    int lit(int i) const { return literals[i]; }
    size_t constraint_size() const { return literals.size(); }

    // Declaration of inline function. Provide definition below or in a source file.
    inline PBConstraintState next_not_false(bool is_left_watch, Lit other_watch, bool binary, int& loc);

    void print_constraint() const {
        for (size_t i = 0; i < literals.size(); ++i) {
            std::cout << coefficients[i] << "*x" << literals[i] << " + ";
        }
        std::cout << "<= " << rhs << std::endl;
    }

    void print_with_watches() const {
        for (size_t i = 0; i < literals.size(); ++i) {
            std::cout << l2rl(literals[i]);
            if (i == lw) std::cout << "L";
            if (i == rw) std::cout << "R";
            std::cout << " ";
        }
        std::cout << "<= " << rhs << std::endl;
    }
};

//-------------------------------------------------------------
// Rename Clause to PBClause (for PB solver)
//-------------------------------------------------------------
class PBClause {
    clause_t c;
    int lw, rw; // watched literal indices	
public:
    PBClause() {}
    void insert(int i) { c.push_back(i); }
    void lw_set(int i) { lw = i; }
    void rw_set(int i) { rw = i; }
    clause_t& cl() { return c; }
    int get_lw() { return lw; }
    int get_rw() { return rw; }
    int get_lw_lit() { return c[lw]; }
    int get_rw_lit() { return c[rw]; }
    int lit(int i) { return c[i]; }

    // Declaration of inline function. Provide definition below or in a source file.
    inline ClauseState next_not_false(bool is_left_watch, Lit other_watch, bool binary, int& loc);

    size_t size() { return c.size(); }
    void reset() { c.clear(); }
    void print() {
        for (clause_it it = c.begin(); it != c.end(); ++it)
            std::cout << *it << " ";
    }
    void print_real_lits() {
        Lit l;
        std::cout << "(";
        for (clause_it it = c.begin(); it != c.end(); ++it) {
            l = l2rl(*it);
            std::cout << l << " ";
        }
        std::cout << ")";
    }
    void print_with_watches() {
        for (clause_it it = c.begin(); it != c.end(); ++it) {
            std::cout << l2rl(*it);
            int j = std::distance(c.begin(), it);
            if (j == lw) std::cout << "L";
            if (j == rw) std::cout << "R";
            std::cout << " ";
        }
    }
};

// Example inline definition for PBClause::next_not_false (replace with your logic):
inline ClauseState PBClause::next_not_false(bool is_left_watch, Lit other_watch, bool binary, int& loc) {
    return ClauseState::C_UNDEF;
}

//-------------------------------------------------------------
// PBSolver class definition
//-------------------------------------------------------------
class PBSolver {
    vector<Constraint> unaries;
    trail_t trail_pb;
    vector<int> separators;
    vector<int> LitScore;
    vector<vector<int>> watches;
    vector<VarState> state_pb;
    vector<VarState> prev_state_pb;
    vector<int> antecedent;
    vector<bool> marked;
    vector<int> dlevel_pb;
    vector<int> conflicts_at_dl;
    vector<int> reason_pb;
    vector<Constraint> pbConstraints;
    unordered_map<int, vector<int>> var_to_pb_constraints;
    unordered_map<int, int> var_occurrence_count;
    unordered_set<int> assigned_vars;
    unordered_set<string> constraint_set;

    map<double, unordered_set<Var>, greater<double>> m_Score2Vars;
    map<double, unordered_set<Var>, greater<double>>::iterator m_Score2Vars_it;
    unordered_set<Var>::iterator m_VarsSameScore_it;
    vector<double> m_activity;
    double m_var_inc;
    double m_curr_activity;
    bool m_should_reset_iterators;
    bool unsat = false;

    unsigned int nvars, nconstraints, noldconstraints, nlits, qhead;
    int num_learned, num_decisions, num_assignments, num_restarts, dl_pb, max_dl, conflicting_constraint_idx, restart_threshold, restart_lower, restart_upper;

    Lit asserted_lit;
    float restart_multiplier;

    int get_learned() { return num_learned; }
    void set_nvars(int x) { nvars = x; }
    int get_nvars() { return nvars; }
    void set_nconstraints(int x) { nconstraints = x; }
    void set_noldconstraints(int x) { noldconstraints = x; }
    size_t pbConstraints_size() { return pbConstraints.size(); }
    VarState get_state_pb(int x) { return state_pb[x]; }
    void set_state_pb(int x, VarState y) { state_pb[x] = y; }

    // misc.
    void add_to_trail(int x) { trail_pb.push_back(x); }

    void reset();
    void initialize();
    void reset_iterators(double activity_key = 0.0);
    SolverState decide();
    void test();
    SolverState BCP();
    int analyze(const Constraint& conflictConstraint);
    inline int getVal(Var v);
    inline void add_constraint(Constraint& c, int l, int r);
    inline void add_unary_constraint(Constraint c);
    inline void assert_lit(Lit l);
    void m_rescaleScores(double& new_score);
    inline void backtrack(int k);
    void restart();
    void normalizePBConstraint(Constraint& pb_constraint, bool bigger);
    Constraint findConflictSubset(Constraint constraint);
    void print_variable_count(int var);
    void processConstraint(const std::string& line, int constraint_index);
    inline void bumpVarScore(int idx);
    inline void bumpLitScore(int lit_idx);

public:
    PBSolver() :
        nvars(0), nconstraints(0), num_learned(0), num_decisions(0), num_assignments(0),
        num_restarts(0), m_var_inc(1.0), qhead(0),
        restart_threshold(Restart_lower), restart_lower(Restart_lower),
        restart_upper(Restart_upper), restart_multiplier(Restart_multiplier)
    {
    }

    inline LitState lit_state(Lit l) {
        VarState var_state = state_pb[l2v(l)];
        return var_state == VarState::V_UNASSIGNED ? LitState::L_UNASSIGNED :
            ((Neg(l) && var_state == VarState::V_FALSE) || (!Neg(l) && var_state == VarState::V_TRUE)) ?
            LitState::L_SAT : LitState::L_UNSAT;
    }
    inline LitState lit_state(Lit l, VarState var_state) {
        return var_state == VarState::V_UNASSIGNED ? LitState::L_UNASSIGNED :
            ((Neg(l) && var_state == VarState::V_FALSE) || (!Neg(l) && var_state == VarState::V_TRUE)) ?
            LitState::L_SAT : LitState::L_UNSAT;
    }
    void read_pb(ifstream& in);
    SolverState _solve();
    void solve();
    void validate_assignment();

    void print_stats() {
        cout << endl << "Statistics: " << endl << "===================" << endl <<
            "### Restarts:\t\t" << num_restarts << endl <<
            "### Learned-clauses:\t" << num_learned << endl <<
            "### Decisions:\t\t" << num_decisions << endl <<
            "### Implications:\t" << num_assignments - num_decisions << endl <<
            "### Time:\t\t" << cpuTime() - begin_time << endl;
    }

    void print_state(const char* file_name) {
        ofstream out(file_name);
        out << "State: ";
        for (auto it = state_pb.begin() + 1; it != state_pb.end(); ++it) {
            char sign = (*it) == VarState::V_FALSE ? -1 : (*it) == VarState::V_TRUE ? 1 : 0;
            out << sign * (it - state_pb.begin()) << " ";
            out << endl;
        }
    }

    void print_state() {
        cout << "State: ";
        for (auto it = state_pb.begin() + 1; it != state_pb.end(); ++it) {
            char sign = (*it) == VarState::V_FALSE ? -1 : (*it) == VarState::V_TRUE ? 1 : 0;
            cout << sign * (it - state_pb.begin()) << " ";
            cout << endl;
        }
    }

    void print_watches() {
        for (auto it = watches.begin() + 1; it != watches.end(); ++it) {
            cout << distance(watches.begin(), it) << ": ";
            for (auto it_c = (*it).begin(); it_c != (*it).end(); ++it_c) {
                pbConstraints[*it_c].print_constraint();
                cout << "; ";
            }
            cout << endl;
        }
    }
};
