#include <bits/stdc++.h>
#include "State.h"

using namespace std;

pair<int, State> MINMAX(const State &state, int depth) {
    if (depth == 0 || state.isFinalState() != -1)
        return {state.getStateCost(), state};

    vector<State> nextStates = state.getNextStates();

    State bestNext;
    int bestCost;

    if (state.turn == 1) {
        bestCost = -10;
        for (auto it: nextStates) {
            auto [costForIt, aux] = MINMAX(it, depth - 1);
            if (bestCost < costForIt) {
                bestNext = it;
                bestCost = costForIt;
            }
        }
    } else {
        bestCost = 10;
        for (auto it: nextStates) {
            auto [costForIt, aux] = MINMAX(it, depth - 1);
            if (bestCost > costForIt) {
                bestNext = it;
                bestCost = costForIt;
            }
        }
    }
    return {bestCost, bestNext};
}

int main() {
    State myGame;
    while (myGame.isFinalState() == -1) {
        if (myGame.turn == 0) {
            myGame.printPlayerNumbers(0);
            int number;
            cin >> number;
            auto pos = State::getPosFromNr(number);
            if (!myGame.move(pos))
                cout << "Wrong move, try again\n";
        } else {
            auto [cost, nextGame] = MINMAX(myGame, 1);
            myGame = nextGame;
            myGame.printPlayerNumbers(1);
            cout << '\n';
        }
    }

    int winner = myGame.isFinalState();
    switch (winner) {
        case 0:
            cout << "Player A won";
            break;
        case 1:
            cout << "Player B won";
            break;
        default:
            cout << "Draw";
    }
    return 0;
}
