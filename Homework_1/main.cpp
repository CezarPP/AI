#include <bits/stdc++.h>
#include "State.h"

using namespace std;
///////////////////////////////////////// 2
/// Initialization function, pass instance, get state

[[nodiscard]] constexpr State getStateFromProblemInstance(const std::span<const int, 9> instance) {
    State state{.lastMoved{-1, -1}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            state.m[i][j] = instance[i * 3 + j];
        }
    }
    return state;
}

[[nodiscard]] constexpr auto getSolutionForEmptyPos(int x, int y) noexcept {
    matrix m{};
    int cnt = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (i != x || j != y)
                m[i][j] = ++cnt;
    return m;
}

[[nodiscard]] constexpr auto getFinalSolutions() noexcept {
    std::array<matrix, 9> solutions{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            // iterate the position of the empty space
            solutions[i * 3 + j] = getSolutionForEmptyPos(i, j);
        }
    return solutions;
}

void printSolution(const matrix &m) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            std::cout << m[i][j] << ' ';
        std::cout << '\n';
    }
    std::cout << '\n';
}

constexpr auto solutions = getFinalSolutions();

/// Check state is final

[[nodiscard]] constexpr bool isFinalState(const State &sol) noexcept {
    return std::ranges::any_of(solutions, [sol](matrix x) {
        return x == sol.m;
    });
}

///////////////////////////////////////// 3

/// Transition validation

[[nodiscard]] constexpr bool inMatrix(std::pair<int, int> pos) noexcept {
    return pos.first >= 0 && pos.first < 3 && pos.second >= 0 && pos.second < 3;
}

[[nodiscard]] constexpr bool isValidTransition(const State &S, const Transition &T) noexcept {
    return inMatrix(T.from) && inMatrix(T.to)
           && S.lastMoved != T.to && S.lastMoved != T.from
           && (S.m[T.from.first][T.from.second] == 0 || S.m[T.to.first][T.to.second] == 0);
}

/// Function that gets state & transition parameters and returns a new state

[[nodiscard]] constexpr State getNewState(const State &S, const Transition &T) noexcept {
    assert(isValidTransition(S, T));
    State newState = S;
    newState.applyTransition(T);
    return newState;
}

[[nodiscard]] constexpr std::vector<State> getReachableStates(const State &s) noexcept {
    std::vector<State> v;
    std::pair<int, int> zeroPos;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (s.m[i][j] == 0)
                zeroPos = {i, j};
    const std::array dx = {1, -1, 0, 0};
    const std::array dy = {0, 0, 1, -1};
    for (int i = 0; i < 4; i++) {
        int newX = zeroPos.first + dx[i], newY = zeroPos.second + dy[i];
        auto transition = Transition{.from = zeroPos, .to = {newX, newY}};
        if (isValidTransition(s, transition))
            v.push_back(getNewState(s, transition));
    }
    return v;
}

[[nodiscard]] constexpr bool isNoneState(const State &state) noexcept {
    if (state.lastMoved.first == 10)
        return true;
    return false;
}


///////////////////////////////////////// Helps print the solution
void
printFoundSolution(const tuple<State, vector<State>, int> &solutionTuple, const char *name) {
    auto [solution, moveSequence, time] = solutionTuple;
    if (!isNoneState(solution)) {
        cout << "Algorithm name " << name << '\n';
        cout << "Number of moves is " << moveSequence.size() << '\n';
        cout << "Elapsed time is " << time << "ms" << '\n';
        if (moveSequence.size() < 10) {
            for (auto it: moveSequence)
                printSolution(it.m);
        } else printSolution(solution.m);
    } else std::cout << "Did not find solution\n";
}

///////////////////////////////////////// Helps to reconstruct the path
vector<State>
getMoveSequence(map<State, State> &m, const State &crtState, const State &initState) {
    vector<State> moves;
    State state = crtState;
    while (state != initState) {
        moves.push_back(state);
        state = m[state];
    }
    moves.push_back(initState);
    std::reverse(moves.begin(), moves.end());
    return moves;
}
///////////////////////////////////////// 4

std::map<State, State> visited;
constexpr static auto noneState = State{.lastMoved{10, 10}};

[[nodiscard]] State limitedDepthDFS(const State &state, int depth) noexcept {
    if (isFinalState(state))
        return state;
    if (depth == 0)
        return noneState;
    auto neighbours = getReachableStates(state);
    for (const auto &it: neighbours)
        if (!visited.contains(it)) {
            visited[it] = state;
            auto res = limitedDepthDFS(it, depth - 1);
            if (!isNoneState(res))
                return res;
        }
    return noneState;
}

tuple<State, vector<State>, int>
IDDFS(const State &initState, int maxDepth) {
    auto start = chrono::high_resolution_clock::now();
    for (int depth = 0; depth < maxDepth; depth++) {
        visited.clear();
        visited[initState] = initState;
        auto sol = limitedDepthDFS(initState, depth);

        if (!isNoneState(sol)) {
            auto moveSequence = getMoveSequence(visited, sol, initState);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            return make_tuple(sol, moveSequence, duration.count());
        }
    }
    throw exception();
}

void printSolutionForInstanceIDDFS(const std::span<const int, 9> instance) {
    const static int MAX_DEPTH = 10'000;
    auto initState =
            getStateFromProblemInstance(instance);
    auto solution = IDDFS(initState, MAX_DEPTH);
    printFoundSolution(solution, "IDDFS");
}

int getManhattanDistanceFromFinalState(const matrix &state, const matrix &finalState) {
    pair<int, int> posFinal[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            posFinal[finalState[i][j]] = {i, j};

    int dist = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            auto pos = posFinal[state[i][j]];
            dist = dist + abs(i - pos.first) + abs(j - pos.second);
        }
    return dist;
}

int getEuclideanDistanceFromFinalState(const matrix &state, const matrix &finalState) {
    pair<int, int> posFinal[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            posFinal[finalState[i][j]] = {i, j};
    int dist = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            auto pos = posFinal[state[i][j]];
            dist += (i - pos.first) * (i - pos.first)
                    + (j - pos.second) * (j - pos.second);
        }
    return dist;
}

int getHammingDistanceFromFinalState(const matrix &state, const matrix &finalState) {
    int dist = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (state[i][j] != finalState[i][j])
                dist++;
    return dist;
}

template<typename Func>
concept StateComparingFunction = std::is_invocable_v<Func, const State &, const State &> &&
                                 std::is_same_v<std::invoke_result_t<Func, const State &, const State &>, bool>;

// The final state, transitions and the execution length
template<StateComparingFunction F>
tuple<State, vector<State>, int>
greedy(const State &initState, const F &f) {
    auto start = chrono::high_resolution_clock::now();
    priority_queue<State, vector<State>, F> q(f);
    q.push(initState);
    map<State, State> m; // distance from the initial state and previous state
    m[initState] = initState;

    while (!q.empty()) {
        auto state = q.top();
        q.pop();

        if (isFinalState(state)) {
            auto moveSequence = getMoveSequence(m, state, initState);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            return make_tuple(state, moveSequence, duration.count());
        }
        auto neighbours = getReachableStates(state);
        for (const auto &it: neighbours)
            if (!m.contains(it)) {
                m[it] = state;
                q.push(it);
            }
    }
    throw exception();
}

template<typename Func>
concept MatrixDistanceF = std::is_invocable_v<Func, const matrix &, const matrix &> &&
                          std::is_same_v<std::invoke_result_t<Func, const matrix &, const matrix &>, int>;

template<typename Func>
concept StateDistanceFromFinalF = std::is_invocable_v<Func, const State &> &&
                                  std::is_same_v<std::invoke_result_t<Func, const State &>, int>;

vector<State> getMoveSequenceAStar(map<State, pair<State, int>> &m, const State &crtState, const State &initState) {
    vector<State> moves;
    State state = crtState;
    while (state != initState) {
        moves.push_back(state);
        state = m[state].first;
    }
    moves.push_back(initState);
    std::reverse(moves.begin(), moves.end());
    return moves;
}

// The final state, transitions and the execution length
template<StateDistanceFromFinalF F>
tuple<State, vector<State>, int>
AStarAlgorithm(const State &initState, const F &f) {
    auto start = chrono::high_resolution_clock::now();

    map<State, pair<State, int>> m; // distance from the initial state and previous state
    m[initState] = {initState, 0};

    auto comparingFunction = [&m, &f](const State &state1, const State &state2) {
        return f(state1) + m[state1].second > f(state2) + m[state2].second;
    };

    priority_queue<State, vector<State>, decltype(comparingFunction)> q(comparingFunction);
    q.push(initState);

    while (!q.empty()) {
        auto state = q.top();
        q.pop();
        int crtDist = m[state].second;

        if (isFinalState(state)) {
            auto moveSequence = getMoveSequenceAStar(m, state, initState);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            return make_tuple(state, moveSequence, duration.count());
        }
        auto neighbours = getReachableStates(state);
        for (const auto &it: neighbours)
            if (!m.contains(it)) {
                m[it] = {state, crtDist + 1};
                q.push(it);
            }
    }
    throw exception();
}

template<MatrixDistanceF F>
auto makeStateComparer(F f) {
    return [dist = f](const State &state1, const State &state2) -> bool {
        int dist1 = 10000, dist2 = 10000;
        for (const auto &it: solutions) {
            dist1 = min(dist1, dist(state1.m, it));
            dist2 = min(dist2, dist(state2.m, it));
        }
        return dist1 > dist2;
    };
}

template<MatrixDistanceF F>
auto makeDistanceFunction(F f) {
    return [dist = f](const State &state1) -> int {
        int dist1 = 10000;
        for (const auto &it: solutions)
            dist1 = min(dist1, dist(state1.m, it));
        return dist1;
    };
}

void printSolutionForGreedy(const std::span<const int, 9> instance) {
    auto initState =
            getStateFromProblemInstance(instance);

    auto hamming = makeStateComparer(getHammingDistanceFromFinalState);
    auto euclidean = makeStateComparer(getEuclideanDistanceFromFinalState);
    auto manhattan = makeStateComparer(getManhattanDistanceFromFinalState);

    auto solutionHamming = greedy(initState, hamming);
    printFoundSolution(solutionHamming, "Greedy Hamming distance");

    auto solutionEuclidean = greedy(initState, euclidean);
    printFoundSolution(solutionEuclidean, "Greedy Euclidean distance ");

    auto solutionManhattan = greedy(initState, manhattan);
    printFoundSolution(solutionManhattan, "Greedy Manhattan distance");
}

void printSolutionForAStar(const std::span<const int, 9> instance) {
    auto initState =
            getStateFromProblemInstance(instance);

    auto hamming = makeDistanceFunction(getHammingDistanceFromFinalState);
    auto euclidean = makeDistanceFunction(getEuclideanDistanceFromFinalState);
    auto manhattan = makeDistanceFunction(getManhattanDistanceFromFinalState);

    auto solutionHamming = AStarAlgorithm(initState, hamming);
    printFoundSolution(solutionHamming, "A* Hamming distance");

    auto solutionEuclidean = AStarAlgorithm(initState, euclidean);
    printFoundSolution(solutionEuclidean, "A* Euclidean distance ");

    auto solutionManhattan = AStarAlgorithm(initState, manhattan);
    printFoundSolution(solutionManhattan, "A* Manhattan distance");
}

int main() {
    const std::array instance1{8, 6, 7, 2, 5, 4, 0, 3, 1};
    const std::array instance2{2, 5, 3, 1, 0, 6, 4, 7, 8};
    const std::array instance3{2, 7, 5, 0, 8, 4, 3, 1, 6};

    printSolutionForGreedy(instance1);
    printSolutionForInstanceIDDFS(instance1);

    cout << "////////////////////////////////////////////////////////\n";

    printSolutionForGreedy(instance2);
    printSolutionForInstanceIDDFS(instance2);

    cout << "////////////////////////////////////////////////////////\n";

    printSolutionForGreedy(instance3);
    printSolutionForInstanceIDDFS(instance3);

    cout << "////////////////////////////////////////////////////////\n";
    printSolutionForAStar(instance1);

    cout << "////////////////////////////////////////////////////////\n";
    printSolutionForAStar(instance2);

    cout << "////////////////////////////////////////////////////////\n";
    printSolutionForAStar(instance3);

    return 0;
}
