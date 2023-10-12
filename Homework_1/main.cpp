#include <bits/stdc++.h>


struct Transition {
    std::pair<int, int> from, to;
};

using matrix = std::array<std::array<int, 3>, 3>;

struct State {
    matrix m;
    std::pair<int, int> lastMoved;

    void applyTransition(const Transition &T) {
        std::swap(m[T.from.first][T.from.second],
                  m[T.to.first][T.to.second]);
    }
};

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

[[nodiscard]] constexpr bool isFinalSolution(const matrix &sol) noexcept {
    return std::ranges::any_of(solutions, [sol](matrix x) {
        return x == sol;
    });
}

[[nodiscard]] constexpr bool inMatrix(std::pair<int, int> pos) noexcept {
    return pos.first >= 0 && pos.first < 3 && pos.second >= 0 && pos.second < 3;
}

[[nodiscard]] constexpr bool isValidTransition(const State &S, const Transition &T) noexcept {
    return inMatrix(T.from) && inMatrix(T.to)
           && S.lastMoved != T.to && S.lastMoved != T.from
           && (S.m[T.from.first][T.from.second] == 0 || S.m[T.to.first][T.to.second] == 0);
}

[[nodiscard]] constexpr State getNewState(const State &S, const Transition &T) noexcept {
    assert(isValidTransition(S, T));
    State newState = S;
    newState.applyTransition(T);
    return newState;
}

int main() {
    for (auto sol: solutions)
        printSolution(sol);

    return 0;
}
