#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <thread>
#include <string>

#include "alphabeta.hpp"

Node::Node(std::shared_ptr<Node>& other, bool isCopy)
{    
  if(isCopy) {
      parent = other->parent;
      parentidx = other->parentidx;
      id = other->id;
      alpha = other->alpha;
      beta = other->beta;
      type = other->type;
      depth = other->depth;
      score = other->score;
      mvsfinal = other->mvsfinal;
      mvsleft = other->mvsleft;
      mvstotal = other->mvstotal;
      splitAttempted = other->splitAttempted;

    } else {
      parent = other;
      parentidx = other->mvstotal - other->mvsleft;
      id = other->id;
      id.push_back(parentidx);  // child's number in parent's child list
      alpha = other->alpha;
      beta = other->beta;
      type = static_cast<Type>(Mini - other->type);
      depth = other->depth - 1;
      if(type == Maxi) {
          score = alpha;
        } else {
          score = beta;
        }
      mvsfinal = 0;
      mvsleft = 0;
      mvstotal = 0;
      splitAttempted = false;
      if(depth > 0)
        expand();
    }

}

Node& Node::operator=(const Node& other)
{
  parent = other.parent;
  parentidx = other.parentidx;
  id = other.id;
  alpha = other.alpha;
  beta = other.beta;
  type = other.type;
  depth = other.depth;
  score = other.score;
  mvsfinal = other.mvsfinal;
  mvsleft = other.mvsleft;
  mvstotal = other.mvstotal;
  splitAttempted = other.splitAttempted;

  return *this;
}

// only for root
Node::Node(uint64_t depth1, Type type1, double alpha1, double beta1) :
  depth(depth1),type(type1), alpha(alpha1), beta(beta1) {

  id.push_back(0);
  parentidx = -1;
  parent = nullptr;
  if(type == Maxi) {
      score = alpha;
    } else {
      score = beta;
    }
  mvsfinal = 0;
  mvsleft = 0;
  mvstotal = 0;
  splitAttempted = false;
  if(depth > 0)
    expand();
}

// builds moves, not new nodes. nodes are constructed one at a time in search()
void Node::expand() {

      uint64_t N = 4;
      for(uint64_t i = 0; i < N; ++i) {
          std::string from = "a";
          std::string to = "c";
          std::string strmove = from + std::to_string(i) + to + std::to_string(i);
          moves.push_back(strmove);
      }
      mvsleft = moves.size();
      mvstotal = moves.size();
}

void Tree::eval(uint64_t tt) {

//  std::random_device rd;
//  std::mt19937 mt(rd());
//  std::mt19937 mt;
//  mt.seed(seed);
//  std::uniform_real_distribution<double> unif(-10.0, 10.0);
//  (*paths[tt].top())->score = unif(mt);

  const std::vector<uint64_t> id = (*paths[tt].top())->id;
  double sc = 0;
  for(size_t i = 0; i < id.size(); ++i)
    sc += std::pow(-1, i) * static_cast<int>(id[i]);
  sc += tt;
  (*paths[tt].top())->score = sc;
  return;
}

// (a, b) effect
bool Tree::ab_pruning(uint64_t tt) {

  std::unique_lock<std::mutex> lkN((*paths[tt].top())->mutexN);

  if((*paths[tt].top())->score > (*paths[tt].top())->beta || (*paths[tt].top())->score < (*paths[tt].top())->alpha) {
      lkN.unlock();
      return true;
  }
  lkN.unlock();
  return false;
}

// do we need to prevent further writing by tt thread to its shared node?
// i.e. does it need to be deep copied to shared state? or just copy a, b
//  int64_t level = depth - path1.size() + 1; should level be known if pruned?
// this could create Shared obj and then shared[tt] = obj, then need lock mutexA inside =

void Tree::postToShared(uint64_t tt, bool pruned) {

 // debug("postToShared a: ", tt);

  std::unique_lock<std::mutex> lkA(shared[tt].mutexA);

  if(shared[tt].U == maxthreads || shared[tt].U == 0) { // update if all threads read or it is first store

      shared[tt].pnode = std::make_shared<Node>(*paths[tt].top(), true);
      for(int i = 0; i < shared[tt].isRead.size(); ++i) {
          shared[tt].isRead[i] = 0;
      }
      std::unique_lock<std::mutex> lkT(mutexS);
      shared[tt].isPruned = pruned;
      lkT.unlock();
      shared[tt].isRead[tt] = 1; // no need for posting thread to read shared state
      shared[tt].U = 1;   // U == M (all threads read me, so can update)
  }

  lkA.unlock();
}

// locks! where to call this?
void Tree::getfromShared(uint64_t tt) {

 // debug("getFromShared a: ", tt);

  for(int i = 0; i < shared.size(); ++i) {

      if(!shared[i].pnode)
          continue;

      std::unique_lock<std::mutex> lkA(shared[i].mutexA);

      if(shared[i].isRead[tt] == 1)
        return;

      const std::vector<uint64_t> path1 = (shared[i].pnode)->id;
      const std::vector<uint64_t> path2 = (*paths[tt].top())->id;

      if(path1.size() < path2.size()) {
          if (std::equal(path1.begin(), path1.end(), path2.begin())) {
              if(shared[i].isPruned) {
                  std::unique_lock<std::mutex> lkT(mutexS);
                  pruned[tt] = 1;
                  lkT.unlock();
              } else {
                std::unique_lock<std::mutex> lkN((*paths[tt].top())->mutexN);
                (*paths[tt].top())->alpha = std::max(shared[i].pnode->alpha, (*paths[tt].top())->alpha);
                (*paths[tt].top())->beta = std::min(shared[i].pnode->beta, (*paths[tt].top())->beta);
                lkN.unlock();
              }
            }
        }

      shared[i].isRead[tt] = 1;
      ++shared[i].U;

      lkA.unlock();
  }
}

void Tree::trySplit(uint64_t tt) {

  // debug("trySplit a: ", tt);

  std::unique_lock<std::mutex> lkN((*paths[tt].top())->mutexN);

  if( !(*paths[tt].top())->splitAttempted && (*paths[tt].top())->mvsleft > min_mvsleft
      && (*paths[tt].top())->depth > min_depth && splitsnum < 2 * maxthreads) {
        (*paths[tt].top())->splitAttempted = true;
        lkN.unlock();
        splits.push(*paths[tt].top());
        std::unique_lock<std::mutex> lkF(mutexF);
        ++splitsnum;
        lkF.unlock();
    } else {
      //  (*paths[tt].top())->splitAttempted = true;
        lkN.unlock();
      //  debug("trySplit ?: ", tt);
    }
}

void Tree::moveUp(uint64_t tt, State state) {

  if(state == Pruned) {
    paths[tt].pop();
    return;
  }

  std::unique_lock<std::mutex> lkN((*paths[tt].top())->mutexN);

  double s = (*paths[tt].pop())->score;

  lkN.unlock();

  std::unique_lock<std::mutex> lkN1((*paths[tt].top())->mutexN);

  auto pnode = *paths[tt].top();

  if(pnode->type == Mini) {
      pnode->score = std::min(s, pnode->score);
    } else {
      pnode->score = std::max(s, pnode->score);
    }

  if(state == SearchedFully) {
      if(pnode->type == Mini) {
          pnode->beta = std::min(s, pnode->beta);
        } else {
          pnode->alpha = std::max(s, pnode->alpha);
        }
      ++pnode->mvsfinal;
    }

  pnode.reset();

  lkN1.unlock();
}

void Tree::search(uint64_t tt) {

  // debug("search a: ", tt);

  if((*paths[tt].top())->depth == 0) {
      eval(tt);
      if(ab_pruning(tt)) {
          moveUp(tt, Pruned);
      }
      else {
        moveUp(tt, SearchedFully);
      }
      return;
  }

  while(true) {  // get the next child

      // check for pruning interrupts, where to put this?
      getfromShared(tt);
      std::unique_lock<std::mutex> lkT(mutexS);
      if(pruned[tt]) {
         lkT.unlock();
         return;
      }
      lkT.unlock();

      trySplit(tt);

      std::unique_lock<std::mutex> lkN((*paths[tt].top())->mutexN);
      if((*paths[tt].top())->mvsleft == 0) {
          lkN.unlock();
          break;
      }
      --(*paths[tt].top())->mvsleft;
      lkN.unlock();

      paths[tt].push(std::make_shared<Node>(*paths[tt].top(), false));

      search(tt);

      // check for pruning interrupts, where to put this?
      getfromShared(tt);
      std::unique_lock<std::mutex> lkT1(mutexS);
      if(pruned[tt]) {
         lkT1.unlock();
         return;
      }
      lkT1.unlock();

      if(ab_pruning(tt)) {
            postToShared(tt, true);
            moveUp(tt, Pruned);
            return;
       }
       else {
          trySplit(tt);
       }

    }

  // no more children, moving up

  // check for pruning interrupts, where to put this?
  getfromShared(tt);
  std::unique_lock<std::mutex> lkT(mutexS);
  if(pruned[tt]) {
     lkT.unlock();
     return;
  }
  lkT.unlock();

  if(*paths[tt].top() != root) {
    std::unique_lock<std::mutex> lkN3((*paths[tt].top())->mutexN);
    if((*paths[tt].top())->mvsfinal == (*paths[tt].top())->mvstotal) {  // fully or not searched subtree
        lkN3.unlock();
        postToShared(tt, false);
        moveUp(tt, SearchedFully);  // move up, and update parent's s,a,b or s
    }
    else {
        lkN3.unlock();
        moveUp(tt, SearchedPartially);  // move up, and update parent's s,a,b or s
    }

  }

  return;
}

void Tree::findSplit(uint64_t tt) {

//  can't call here as it has no node if tt > 0
//  debug("findSplit a: ", tt);

    std::unique_lock<std::mutex> lkF(mutexF);
    ++numWaiting;
    lkF.unlock();
    cv_final.notify_one();

    std::shared_ptr<Node> splitnode = nullptr;
    while(!done) {

       auto sp = splits.pop();  //  shared<weak>
       if(sp == nullptr)
         continue;
       splitnode = (*sp).lock(); // shared
       lkF.lock();
       if(splitnode != nullptr) {
          --numWaiting;
       }
       --splitsnum;
       lkF.unlock();
       cv_final.notify_one();
       if(splitnode != nullptr)
           break;
   }

    if(done) {
        return;
    }


    thstack<std::shared_ptr<Node> > local;

    auto pnode = splitnode->parent;
    while(pnode) {
      local.push(pnode);
      pnode = pnode->parent;
    }

    // push path to new search node, no need for lock here
    while(local.size() > 0) {
        paths[tt].push(std::move(*local.pop()));
    }
    paths[tt].push(splitnode);

    splitnode.reset();

    search(tt);

   // debug("findSplit a: ", tt);

  return;
}

void Tree::debug(std::string loc, uint64_t tt) {

  std::cout << loc;
  std::unique_lock<std::mutex> lkB(mutexB);
  std::unique_lock<std::mutex> lkN((*paths[tt].top())->mutexN);

  const std::vector<uint64_t> id = (*paths[tt].top())->id;
  std::cout << tt << "   dflt:  " << (*paths[tt].top())->depth << " " << (*paths[tt].top())->mvsfinal << " " <<
                (*paths[tt].top())->mvsleft  << " " << (*paths[tt].top())->mvstotal << "    asb: " << (*paths[tt].top())->alpha << "  "
            << (*paths[tt].top())->score << "  " << (*paths[tt].top())->beta << "     id: ";
   for(size_t i = 0; i < id.size(); ++i) {
     std::cout << id[i] << "  ";
   }
   std::cout <<  "       wsp: " << numWaiting << "   " << splitsnum << "  " << pruned[tt] << std::endl;
   std::cout << std::endl;

   lkN.unlock();
   lkB.unlock();
}

Shared::Shared(uint64_t M) {

  isRead.reserve(M);
  for(int i = 0; i < M; ++i) {
      isRead.push_back(0);
  }
  pnode = nullptr;
  isPruned = false;
  U = 0;
}

// if we copy Shared we need mutex to be mutable and lock
Shared::Shared(Shared& other)
{
  pnode = other.pnode;
  isRead = other.isRead;
  isPruned = other.isPruned;
  U = other.U;
  //std::cout << "copy called " << "\n";
}

Shared::Shared(Shared&& other) : isRead(std::move(other.isRead)), isPruned(std::move(other.isPruned)),
  U(std::move(other.U)), pnode(std::move(other.pnode))
{
  other.pnode = nullptr;
  //std::cout << "move called " << "\n";
}

Tree::Tree(uint64_t maxthreads1) : maxthreads(maxthreads1), depth(7), type(Maxi)
  ,root(std::make_shared<Node>(depth, type, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()))
{
 // root = std::make_shared<Node>(depth, type, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  width = 3;
  min_mvsleft = 1;
  min_depth = 2;
  splitsnum = 0;
  numWaiting = 0;

  std::cout << (*root).alpha << "  " << (*root).depth << "\n";

  threads.reserve(maxthreads);
  pruned.reserve(maxthreads);
  paths.reserve(maxthreads);
  shared.reserve(maxthreads);

  done = false;

  for(int i = 0; i < maxthreads; ++i) {
      shared.push_back(Shared(maxthreads));
      //std::cout << shared[i].isRead.size() << "\n";
  }

}

void Tree::init_thread(uint64_t tt) {

  // if depth == 0, won't work in moveUp()
  // think if we lose info here by pop() last node from stack
  if(tt == 0) {
      paths[tt].push(root);
      search(tt);
      while(paths[tt].size() > 0)
        paths[tt].pop();
  }

   while(!done) {
     //  std::cout << " recharging: " << tt << "\n";
      std::unique_lock<std::mutex> lkT(mutexS);
      pruned[tt] = 0;
      lkT.unlock();
      findSplit(tt);
      while(paths[tt].size() > 0)
        paths[tt].pop();
   }
}

void Tree::init() {

  std::unique_lock<std::mutex> lkT(mutexS);

  while(threads.size() < maxthreads) {
      pruned.push_back(0);
      thstack<std::shared_ptr<Node> > path;
      paths.push_back(path);
      threads.push_back(std::thread(&Tree::init_thread, this, threads.size()));
      std::cout << "thread started... " << threads.size() << std::endl;
  }

  lkT.unlock();

  std::unique_lock<std::mutex> lkF(mutexF);
  cv_final.wait(lkF, [=]{return !splitsnum && numWaiting == threads.size();});
  done = true;
  lkF.unlock();

 // std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Tree::finish() {

  std::unique_lock<std::mutex> lkB(mutexB);
  std::cout << "in finish(), done = true, ";
  std::cout << " numWaiting = " << numWaiting << ",   ";
  std::cout << " splitsnum = " << splitsnum << ",   ";
  std::cout << " root score = " << root->score << ",   ";
  std::cout << " numthreads = " << threads.size() << std::endl;
  lkB.unlock();

  for(int i = 0; i < threads.size(); i++) {
       if(threads[i].joinable()) {
           std::cout << "joining... " << i+1 << std::endl;
          threads[i].join();
        }
    }
}

/*
 * minimax test using TT-like hash table (zobrist)
 *
 * probabilistic node split
 *
 *  call node constructor from another
 *
 * add relative level to search()
 *
 * cleanup code in destructor only
 *
 * bool in object ...
 *
 * add noexcept to functions
 *
 * padding in vector of Shared to minimize false sharing
 *
 * Check tree mutexes
 *
 * remove mutexN from Node ? No.But paths pops and pushes don't need locks. Only when access node's data
 * because two threads can be doing that.
 *
 * defer lock, then lock(m1,m2) can avoid deadlock
 *
 * check what is pushed onto thstack, push uses copy/move constructors. it is ok since shared_ptr is pushed
 *
 * postToShared() needs = operator
 *
 * trySplit()
 *
 */



