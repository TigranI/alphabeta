#ifndef ALPHABETA_HPP
#define ALPHABETA_HPP

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "thstack.hpp"

enum State {
  Pruned = 1,
  SearchedFully = 2,
  SearchedPartially = 3
};

enum Type {
  Maxi = 0,
  Mini = 1
};

class  Node {

public:
 std::vector<uint64_t> id;
 uint64_t parentidx;
 uint64_t mvsleft;
 uint64_t mvstotal;
 uint64_t mvsfinal;
 bool splitAttempted;
 Type type;
 double alpha;
 double beta;
 double score;
 uint64_t depth;
 std::vector<std::string> moves;

 std::mutex mutexN;
 std::shared_ptr<Node> parent;

 Node(std::shared_ptr<Node>&, bool);
 Node(uint64_t, Type, double, double);
 Node& operator=(const Node&);
 void expand();
};

class Shared {

public:
  std::shared_ptr<Node> pnode;
  vector<char> isRead;
  bool isPruned;
  uint64_t U;

  std::mutex mutexA;

  Shared(Shared&);
  Shared(Shared&&);
  Shared(uint64_t);
};

// main class
class Tree {

public:

  std::mutex mutexB;
  std::mutex mutexS;
  std::mutex mutexF;

   uint64_t min_mvsleft;
   uint64_t min_depth;
   uint64_t depth;
   uint64_t width;

  std::condition_variable cv_final;
  bool done;
  uint64_t numWaiting;
  thstack<std::weak_ptr<Node> > splits;
  uint64_t splitsnum;
  std::shared_ptr<Node> root;
  Type type;  // root type
  uint64_t maxthreads;

  std::vector<Shared> shared;
  vector<int64_t> pruned;
  vector<thstack<std::shared_ptr<Node> > > paths;
  std::vector<std::thread> threads;

  Tree(uint64_t);
  void init();
  void init_thread(uint64_t);
  void search(uint64_t);
  bool ab_pruning(uint64_t);
  void ab_update_below(uint64_t);
  void ab_prune_below(uint64_t);
  void moveUp(uint64_t, State);
  void eval(uint64_t);
  void trySplit(uint64_t);
  void findSplit(uint64_t);
  void postToShared(uint64_t, bool);
  void getfromShared(uint64_t);
  void finish();

  void debug(std::string, uint64_t);
};

#endif // ALPHABETA_HPP
