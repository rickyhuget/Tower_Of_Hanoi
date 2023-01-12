#include <iostream>
#include <stdlib.h> /* to use exit */
#include <string>
#include <vector>
using namespace std;

//====================================================
// YOUR NAME: Ricky Huget
//====================================================

class state {
public:
  string _left; // left, mid, and right represent the pegs and the indexes
                // represent a disk and
  string _mid; // its respective size the leftmost (index 0) portrays the top of
               // the disk stack
  string _right;    // state is the overall representation of the disks and
  string _state;    // pegs with '|' char dividing positions (pegs)
  string _camefrom; // camefrom is the previous state
  int g;            // cost so far
  int h;            // estimated cost to the goal
  int f;            // g + h, "goodness" of the state

  state(string disks, int numDisks) {
    _left = disks; // all disks start on left peg
    _mid = "";
    _right = "";
    _state = "{ " + _left + "|" + _mid + "|" + _right + " }";
    g = 0;        // initial cost
    h = numDisks; // amount of disks assigned
    f = g + h;
  }
};

// Major data structures
vector<state> frontier;  // Frontier nodes
vector<state> beenThere; // already expanded nodes
// Global variables
string STACK;
string ENDSTATE;

//============Utility Functions==============

// utility to add x to the beenthere
void addtoBeenThere(state x) { beenThere.push_back(x); }

// utility to add x to the frontier
void addtoFrontier(state x) { frontier.push_back(x); }

// utility to check if state has already been encountered
bool duplicateState(state x) {
  for (int i = 0; i < beenThere.size(); i++) {
    if (x._state == beenThere[i]._state)
      return true;
  }
  return false;
}

// utility to check if state is unsafe
bool illegalState(state x) {
  if ((x._left.length() > 1) &&
          (x._left[0] > x._left[1]) || // if disk on top is greater than
      (x._mid.length() > 1) && (x._mid[0] > x._mid[1]) || // the disk below it
      (x._right.length() > 1) &&
          (x._right[0] > x._right[1])) { // then state is illegal
    return true;
  }
  return false;
}

// to remove state x from the frontier
void removefromFrontier(state x) {
  vector<state> newfrontier;
  for (int k = 0; k < frontier.size(); k++)
    if (frontier[k]._state != x._state)
      newfrontier.push_back(frontier[k]);
  frontier = newfrontier;
}

int hCaveat(state s) {
  if ((s._left.length() > 1) &&
          ((s._left[0] + 1) < s._left[1]) || // if disk on top is
      (s._mid.length() > 1) &&
          ((s._mid[0] + 1) < s._mid[1]) || // two sizes smaller
      (s._right.length() > 1) &&
          ((s._right[0] + 1) < s._right[1])) { // then h += 2
    return 2;
  }
  return 0;
}

void updateState(state &next) {
  next._state = "{ " + next._left + "|" + next._mid + "|" + next._right + " }";
}

//==============For Generating a New State================

// Trace and display the solution path from goal to initial.
// Note that camefrom of each state should be used. (called from generate)

void tracePath(state goal) { // from generate function
  // using cameFroms' -- recursion would be useful to not to depend on a
  // particular order of state in Been There!
  state current = goal; // copies goal into current state

  cout << "\nGoal is: " << goal._state << endl; // output for user
  cout << "Path from goal:" << endl;            // output for user

  for (int i = beenThere.size() - 1; i >= 0;
       i--) { // loop for checking to seen which states match beenThere
    if (beenThere[i]._state ==
        current._camefrom) { // if camefrom is the same as beenThere, then it
                             // displays the item
      cout << beenThere[i]._state << endl << "came from" << endl;
      current = beenThere[i]; // updates the beenThere just found with the
                              // current to search again
    }
  }
  cout << "Finished!" << endl; // user message
} // end of tracePath

// Check to see if next's items is already in frontier and return true or false.
// If it is already in frontier,
// and if the next's f is better,
// update the frontier node to next. (called from generate)
bool inFrontier(state next) {
  // the same frontier node with next if f is better.
  // Please cout a message in that case.
  // Return true or false.
  for (size_t i = 0; i < frontier.size();
       i++) { // loop for checking the frontier vector
    if (frontier[i]._state == next._state) {
      return true;
    } // if 'next' is already in frontier
  }
  return false; // if 'next' is not in frontier
} // end of inFrontier

// Try to add next to frontier but stop if goal reached
void generate(state next) {
  updateState(next); // update _state
  cout << "Trying to add: " << next._state;

  if (next._state == ENDSTATE) { // the goal is reached
    addtoBeenThere(next);
    cout << " >> Goal Reached!" << endl;
    tracePath(next); // display the solution path
    exit(0);
  } // done

  // if been there before, do not add to frontier.
  if (duplicateState(next)) {
    cout << " => Legel. But a previous state." << endl;
    removefromFrontier(next);
    return;
  }

  // if unsafe, do not add to frontier.
  if (illegalState(next)) {
    cout << " => Illegal." << endl;
    removefromFrontier(next);
    return;
  }

  // else compute h and then f for next
  // int h = 0; // start point for h; g is already set
  // update h and then f
  else {
    next.h = next._left.size() + next._mid.size() +
             hCaveat(next);   // h is the number of disks not in _right and
                              // hCaveat function
    next.f = next.h + next.g; // update f for this state
  }

  if (!inFrontier(next)) {
    cout << " => Legal. Adding to frontier." << endl;
    addtoFrontier(next); // add next to Frontier
  }
} // end of generate

// Generate all new states from current (called from main)
void generateAll(state current) {
  frontier.clear();                   // clears frontier for new expansion
  current.g += 1;                     // to update g to be used for next
  current._camefrom = current._state; // storing the parent so that we can
                                      // produce the solution path

  // each next will be modified from current for ease
  state next = current; // a generated state & starting point of next is current
  // check all possibilities:
  // move top of left to top of mid
  if (current._left != "") { // not if empty
    next._mid =
        current._left[0] + current._mid; // put top of former on top of latter
    next._left =
        current._left.substr(1, current._left.size() - 1); // take off index 0
    generate(next);
  }
  // move top of left to top of right
  next = current;            // starting point of next is current
  if (current._left != "") { // not if empty
    next._right =
        current._left[0] + current._right; // put top of former on top of latter
    next._left =
        current._left.substr(1, current._left.size() - 1); // take off index 0
    generate(next);
  }
  // move top of mid to top of mid
  next = current;           // starting point of next is current
  if (current._mid != "") { // not if empty
    next._left =
        current._mid[0] + current._left; // put top of former on top of latter
    next._mid =
        current._mid.substr(1, current._mid.size() - 1); // take off index 0
    generate(next);
  }
  // move top of mid to top of mid
  next = current;           // starting point of next is current
  if (current._mid != "") { // not if empty
    next._right =
        current._mid[0] + current._right; // put top of former on top of latter
    next._mid =
        current._mid.substr(1, current._mid.size() - 1); // take off index 0
    generate(next);
  }
  // move top of right to top of mid
  next = current;             // starting point of next is current
  if (current._right != "") { // not if empty
    next._left =
        current._right[0] + current._left; // put top of former on top of latter
    next._right =
        current._right.substr(1, current._right.size() - 1); // take off index 0
    generate(next);
  }
  // move top of right to top of mid
  next = current;             // starting point of next is current
  if (current._right != "") { // not if empty
    next._mid =
        current._right[0] + current._mid; // put top of former on top of latter
    next._right =
        current._right.substr(1, current._right.size() - 1); // take off index 0
    generate(next);
  }
} // end of generateAll

// Find the best f state of the frontier and return it (called from main)
state bestofFrontier() {
  int betterState_f =
      2147483647; // set better state to highest int value in order to work down
  int betterState_position = 0; // better state position in the frontier vector
  for (int i = 0; i < frontier.size(); i++) {
    if (frontier[i].f <
        betterState_f) { // lowest f value for states is desireable (if tie,
                         // chooses state on the left)
      betterState_f = frontier[i].f;
      betterState_position = i; // marks the position of the better state
    }
  }
  cout << "Choosing state: " << frontier[betterState_position]._state << endl;
  return frontier[betterState_position]; // returns the state with lowest f
                                         // value in frontier
} // end of bestofFrontier

// Display the states in the frontier (called from MAIN)
void displayFrontier() {
  for (int k = 0; k < frontier.size(); k++) {
    cout << frontier[k]._state << " ";
    cout << "g = " << frontier[k].g << " ";
    cout << "h = " << frontier[k].h << " ";
    cout << "f = " << frontier[k].f << endl;
  }
} // end of displayFrontier

void printVect(std::vector<state> const &v) {
  cout << "Vector is : ( ";
  for (int i = 0; i < v.size(); i++) {
    cout << v.at(i)._state << " ";
  }
  cout << ")" << endl;
}

//=======================MAIN==========================
int main() {
  STACK = "123"; // stack of disks used
  ENDSTATE = "{ ||" + STACK +
             " }"; // desired end state is all disks on state's right peg

  state current(STACK, STACK.length()); // initial state, length of STATE will
                                        // correspond to moves need to goal (h)

  // updateState(current);
  addtoFrontier(current);

  char proceed; // arbitrary char for before loop
  do {
    removefromFrontier(current);
    addtoBeenThere(current);

    cout << ">>>Expand: " << current._state << endl;
    generateAll(current); // new states are added to frontier
    cout << "Frontier is:" << endl;
    displayFrontier();

    current = bestofFrontier(); // pick the best state out of the frontier

    cout << "\nEnter 'q' to quit.\nEnter any other key to continue." << endl;
    cin >> proceed;
    cout << endl;
    if (proceed == 'q') {
      break;
    }

  } while (proceed != 'q');

  return 0;
} // End of MAIN