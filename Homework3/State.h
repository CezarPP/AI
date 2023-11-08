#pragma once

#include <bits/stdc++.h>

using namespace std;

template<typename T>
using TableType = array<array<T, 4>, 4>;

struct State {
    constexpr static array numbers{2, 7, 6,
                                   9, 5, 1,
                                   4, 3, 8};

    static pair<int, int> getPosFromNr(int nr) {
        int p;
        p = (int) (find(numbers.begin(), numbers.end(), nr) - numbers.begin());
        return make_pair(p / 3 + 1, p % 3 + 1);
    }

    static int getNrFromPos(pair<int, int> pos) {
        int x = (pos.first - 1) * 3, y = pos.second - 1;
        return numbers[x + y];
    }


    TableType<int> state{};
    int turn; // 0 for A 1 for B

    State() {
        turn = 0;
        for (int i = 1; i <= 3; i++)
            for (int j = 1; j <= 3; j++)
                state[i][j] = -1;
    }


    [[nodiscard]] bool isWinState() const {
        //row
        for (int i = 1; i <= 3; i++) {
            bool row = true;
            for (int j = 1; j <= 3; j++)
                if (state[i][j] != state[i][1] || state[i][j] == -1)
                    row = false;

            if (row)
                return true;
        }

        //column
        for (int j = 1; j <= 3; j++) {
            bool col = true;
            for (int i = 1; i <= 3; i++)
                if (state[i][j] != state[1][j] || state[i][j] == -1)
                    col = false;

            if (col)
                return true;
        }

        //main diagonal
        if (state[1][1] == state[2][2] && state[1][1] == state[3][3] && state[1][1] != -1)
            return true;

        //second diagonal
        if (state[1][3] == state[2][2] && state[1][3] == state[3][1] && state[1][3] != -1)
            return true;

        return false;
    }

    [[nodiscard]] int isFinalState() const {
        if (isWinState())
            return 1 - turn; // the previous one who moved won
        else {
            bool full = true;
            for (int i = 1; i <= 3; i++)
                for (int j = 1; j <= 3; j++)
                    if (state[i][j] == -1)
                        full = false;

            if (full)
                return 2;
            return -1;
        }
    }

    bool move(pair<int, int> pos) {
        if (state[pos.first][pos.second] != -1)
            return false;

        state[pos.first][pos.second] = turn;
        turn = 1 - turn;
        return true;
    }

    void printPlayerNumbers(int player) const {
        vector<int> chosen;
        for (int i = 1; i <= 3; i++)
            for (int j = 1; j <= 3; j++)
                if (state[i][j] == player)
                    chosen.push_back(getNrFromPos(make_pair(i, j)));
        sort(chosen.begin(), chosen.end());

        if (player == 0)
            cout << "A: ";
        else cout << "B: ";

        for (auto it: chosen)
            cout << it << ' ';
    }

    [[nodiscard]] vector<State> getNextStates() const {
        vector<State> nextStates;

        State nextState = *this;
        for (int i = 1; i <= 3; i++)
            for (int j = 1; j <= 3; j++)
                if (nextState.move(make_pair(i, j))) {
                    nextStates.push_back(nextState);
                    nextState = *this;
                }
        return nextStates;
    }

    [[nodiscard]] int freedom(int player) const {
        int score = 0;
        // row
        for (int i = 1; i <= 3; i++) {
            bool row = true;
            for (int j = 1; j <= 3; j++)
                if (state[i][j] == 1 - player)
                    row = false;
            if (row)
                score++;
        }

        //col
        for (int j = 1; j <= 3; j++) {
            bool col = true;
            for (int i = 1; i <= 3; i++)
                if (state[i][j] == 1 - player)
                    col = false;

            if (col)
                score++;
        }

        // main diag
        bool diag = true;
        for (int i = 1; i <= 3; i++)
            if (state[i][i] == 1 - player)
                diag = false;

        if (diag)
            score++;


        //second diag
        diag = true;
        for (int i = 1; i <= 3; i++)
            if (state[i][3 - i + 1] == 1 - player)
                diag = false;

        if (diag)
            score++;

        return score;
    }

    [[nodiscard]] int getStateCost() const {
        int winner = isFinalState();

        if (winner == 0)
            return -10;
        if (winner == 1)
            return 10;

        return freedom(1) - freedom(0);
    }

    bool operator<(const State &other) const {
        return getStateCost() < other.getStateCost();
    }

};

